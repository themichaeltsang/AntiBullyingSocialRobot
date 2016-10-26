#include <ros/ros.h> //should be ok
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <map>
#include <unordered_map>
#include <exception>
#include "data_interface/DataLib.h"
#include <chrono>
#include <thread>

#include "kinect_bridge2/KinectBodies.h"
#include "kinect_bridge2/KinectFaces.h"
#include "kinect_bridge2/KinectHDFaces.h"
#include "kinect_bridge2/KinectRawAudio.h"
#include "kinect_bridge2/CLMHeads.h"

#include "multimodal/MultimodalState.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>


using namespace std;

typedef kinect_bridge2::KinectBodies _KinectBodiesMsg;
typedef kinect_bridge2::KinectBody _KinectBodyMsg;
typedef kinect_bridge2::KinectJoint _KinectJointMsg;

typedef kinect_bridge2::KinectFaces _KinectFacesMsg;
typedef kinect_bridge2::KinectFace _KinectFaceMsg;
typedef kinect_bridge2::KinectFaceProperty _KinectFacePropertyMsg;

typedef kinect_bridge2::KinectHDFaces _KinectHDFacesMsg;
typedef kinect_bridge2::KinectHDFace _KinectHDFaceMsg;
typedef kinect_bridge2::KinectHDFaceAnimationUnit _KinectAnimationUnitMsg;

typedef kinect_bridge2::KinectRawAudio _KinectRawAudioMsg;

typedef kinect_bridge2::CLMHeads _CLMHeadsMsg;
typedef kinect_bridge2::CLMHead _CLMHeadMsg;
typedef kinect_bridge2::CLMFacialActionUnit _CLMFacialActionUnitMsg;

typedef multimodal::MultimodalLabel _MultimodalLabelMsg;


//vector<pair<float, string> > abstime_singlepersonid_ordering_;



vector<string> GetFileNames(string directoryPath)
{
    namespace fs = boost::filesystem ;
    vector<string> names ;

    if ( fs::exists(directoryPath) )
    {
        fs::directory_iterator it(directoryPath) ;
        fs::directory_iterator end ;

        while ( it != end )
        {
            names.push_back(it->path().filename().string()) ;
            ++it ;
        }
    }

    return names ;
}

vector<string> FilenamesContainingPattern(vector<string> strings, string pattern)
{
    auto pos = remove_if(begin(strings), end(strings), 
                              [&](string& s) { return s.find(pattern) == string::npos ; }) ; 

    strings.erase(pos, std::end(strings)) ;
    return strings;
}


vector<pair<float, string>> GetAbsTimeSinglePersonIdOrdering(string directorypath, vector<string> filenames)
{

	string single_body_idx_filename = (FilenamesContainingPattern(filenames, "SingleKinectBodyIdxInfo"))[0];
	string filepath2 = directorypath + "/" + single_body_idx_filename;

	//cout << "FILEPATH: " << filepath2 << endl;

	ifstream file(filepath2, ifstream::in); //construct the file stream
	string line;
	int numlines = 0;

	vector<pair<float,string>> abstime_singlepersonid_ordering;

	while(getline(file, line))
	{
		//cout << "bodyidx numlines " << numlines <<endl;
		if(numlines == 0)
		{		
			++numlines;
			continue;
		}
		stringstream ss(line);
		//vector<float> datapoints;
		string token;

		int numcells = 0;
		pair<float,string> abstime_personid_mapping;

		while(getline(ss, token, ','))
		{
			if (numcells == 0)
			{
				abstime_personid_mapping.first = stof(token);
			}
			else if (numcells == 4)
			{
				abstime_personid_mapping.second = token;

			}

			//datapoints.push_back(stof(token));
			//cout << << endl;
			++numcells;
		}

		//cout << "abstime persondid mapping, first " << abstime_personid_mapping.first << "second " << abstime_personid_mapping.second << endl;
		//try to find appropriate mapping in datamap...via incremental search through time... for all items...
		abstime_singlepersonid_ordering.push_back(abstime_personid_mapping);

		//datamap[filename].push_back(datapoints); // all the data
		++numlines;
	}
	return abstime_singlepersonid_ordering;

	//return abstime_singlepersonid_ordering;
}



void ReadDataFromDirectory(vector<vector<float> >& data, map<string, int>& headermap, string directorypath, vector<string> directoryfilenames, string datasource)
{ 
	vector<string> filenames_datasource = FilenamesContainingPattern(directoryfilenames, datasource);

  //TODO: loop the following file reader according to which person comes first...
	//unordered_map<int, unordered_map<int, vector<float> > > datamap;
	//datamap[stdbodyidx] = unordered_map<int, vector<float> >();

	string filepath = directorypath + "/" + filenames_datasource[0];
	//cout << "FILEPATH: " << filepath << endl;
	//cout << "personid to match: " << stoi(personid.substr(6)) << endl;

	ifstream file(filepath, ifstream::in); //construct the file stream
	string line;
	int numlines = 0;

	while(getline(file, line))
	{
		if(numlines == 0)
		{
			stringstream ss1(line);
			string cell;
			int cellcount = 0;
			while(getline(ss1, cell, ','))
			{
			headermap.emplace(make_pair(cell, cellcount)); //this is a problem
			++cellcount;
			}
			++numlines;
			continue;
		}
		stringstream ss(line);
		vector<float> datapoints;
		string token;

		while(getline(ss, token, ','))
		{
			float f = -1.;

			try{
				f = stof(token);
			}
			catch(... )
			{

			}
			datapoints.push_back(f);
		}
		data.push_back(datapoints); // all the data
		++numlines;
	}
}

void ReadDataFromDirectoryMap(unordered_map<float, vector<float> >& data, map<string, int>& headermap, string directorypath, vector<string> directoryfilenames, string datasource)
{ 
	vector<string> filenames_datasource = FilenamesContainingPattern(directoryfilenames, datasource);

  //TODO: loop the following file reader according to which person comes first...
	//unordered_map<int, unordered_map<int, vector<float> > > datamap;
	//datamap[stdbodyidx] = unordered_map<int, vector<float> >();

	string filepath = directorypath + "/" + filenames_datasource[0];
	//cout << "FILEPATH: " << filepath << endl;
	//cout << "personid to match: " << stoi(personid.substr(6)) << endl;

	ifstream file(filepath, ifstream::in); //construct the file stream
	string line;
	int numlines = 0;

	while(getline(file, line))
	{
		if(numlines == 0)
		{
			stringstream ss1(line);
			string cell;
			int cellcount = 0;
			while(getline(ss1, cell, ','))
			{
			headermap.emplace(make_pair(cell, cellcount)); //this is a problem
			++cellcount;
			}
			++numlines;
			continue;
		}
		stringstream ss(line);
		vector<float> datapoints;
		string token;

		while(getline(ss, token, ','))
		{
			float f = -1.;

			try{
				f = stof(token);
			}
			catch(... )
			{

			}
			datapoints.push_back(f);
		}
		data.emplace(make_pair(datapoints[0], datapoints)); // all the data
		++numlines;
	}
}

// void ReadDataFromFile(map<int, vector<float> >& data, map<string, int>& headermap, string filepath, vector<string> directoryfilenames, string datasource)
// { 

// 	ifstream file(filepath, ifstream::in); //construct the file stream
// 	string line;
// 	int numlines = 0;

// 	while(getline(file, line))
// 	{
// 		if(numlines == 0)
// 		{
// 			stringstream ss1(line);
// 			string cell;
// 			int cellcount = 0;
// 			while(getline(ss1, cell, ','))
// 			{
// 			headermap.emplace(make_pair(cell, cellcount)); //this is a problem
// 			++cellcount;
// 			}
// 			++numlines;
// 			continue;
// 		}
// 		stringstream ss(line);
// 		vector<float> datapoints;
// 		string token;

// 		while(getline(ss, token, ','))
// 		{
// 			datapoints.push_back(stof(token));
// 		}
// 		data.push_back(datapoints); // all the data
// 		++numlines;
// 	}
// }

void ReadPersonDataFromDirectory(vector<vector<float> >& data, map<string, int>& headermap, string directorypath, vector<string> directoryfilenames, string datasource, vector<pair<float, string> >& ordering)
{ 

	vector<string> filenames_datasource = FilenamesContainingPattern(directoryfilenames, datasource);

  
  //TODO: loop the following file reader according to which person comes first...
	unordered_map<int, unordered_map<int, vector<float> > > datamap;

	for (string filename : filenames_datasource)
	{

		vector<string> strs;
		boost::split(strs,filename,boost::is_any_of("_"));
		string splitstring = strs[strs.size()-1];
		boost::split(strs,splitstring,boost::is_any_of("."));
		string personid = strs[0];
		float stdbodyidx = stof(personid.substr(6));




		datamap[stdbodyidx] = unordered_map<int, vector<float> >();

		string filepath = directorypath + "/" + filename;
		//cout << "FILEPATH: " << filepath << endl;
		//cout << "personid to match: " << stoi(personid.substr(6)) << endl;



		ifstream file(filepath, ifstream::in); //construct the file stream
		string line;
		int numlines = 0;

		while(getline(file, line))
		{
			if(numlines == 0)
			{
				stringstream ss1(line);
				string cell;
				int cellcount = 0;
				while(getline(ss1, cell, ','))
				{
				headermap.emplace(make_pair(cell, cellcount)); //this is a problem
				++cellcount;
				}
				++numlines;
				continue;
			}
			stringstream ss(line);
			vector<float> datapoints;
			string token;

			while(getline(ss, token, ','))
			{
				datapoints.push_back(stof(token));
			}
			datamap[stdbodyidx][datapoints[0]] = datapoints; // all the data
			++numlines;
		}
	}

	for (auto abstime_personid_mapping : ordering)
	{

		//cout << "abstime persondid mapping, first " << abstime_personid_mapping.first << ", second " << abstime_personid_mapping.second << endl;

		float abstime = abstime_personid_mapping.first;
		vector<string> strs;
		boost::split(strs,abstime_personid_mapping.second,boost::is_any_of("|"));
		int stdbodyidx = stof(strs[1]);

  
		if (datamap.count(stdbodyidx) > 0)
		{
			if (datamap[stdbodyidx].count(abstime) > 0)
			{
				data.push_back(datamap[stdbodyidx][abstime]);
				// cout << "data pushed!" << endl;
			}
		}

	}

}



_KinectBodiesMsg GetKinectBodyMsg(vector<float> featureVectorKinectBodies, map<string, int> headerMapKinectBodies)
{
	_KinectBodiesMsg ros_bodies_msg;
	map<KinectJointData::JointType, string> jointLookUp = KinectJointData::buildJointNamesMap();
	_KinectBodyMsg ros_body_msg;

	ros_body_msg.is_tracked = true;
	ros_body_msg.std_body_idx = featureVectorKinectBodies[headerMapKinectBodies["body_idx"]];

	ros_body_msg.frame = featureVectorKinectBodies[headerMapKinectBodies["frame"]]; // frame
	ros_body_msg.time_stamp = featureVectorKinectBodies[headerMapKinectBodies["time"]]; //include timestamp

	ros_body_msg.lean.x = featureVectorKinectBodies[headerMapKinectBodies["lean_x"]]; // frame
	ros_body_msg.lean.y = featureVectorKinectBodies[headerMapKinectBodies["lean_y"]]; //include timestamp


	ros_body_msg.hand_state_left = static_cast<uint8_t>( featureVectorKinectBodies[headerMapKinectBodies["hand_state_left"]] );
	ros_body_msg.hand_state_right = static_cast<uint8_t>( featureVectorKinectBodies[headerMapKinectBodies["hand_state_right"]] );

	auto & ros_joints_msg = ros_body_msg.joints;
	for(int i = 0; i < 25; ++i)
	{
		_KinectJointMsg ros_joint_msg;
		string jointname = jointLookUp[static_cast<KinectJointData::JointType>(i)];
		float xp, yp, zp, wo, xo, yo, zo;

		xp = featureVectorKinectBodies[headerMapKinectBodies[jointname + "_position_x"]] ;
		yp = featureVectorKinectBodies[headerMapKinectBodies[jointname + "_position_y"]] ;
		zp = featureVectorKinectBodies[headerMapKinectBodies[jointname + "_position_y"]] ;
		
		wo = featureVectorKinectBodies[headerMapKinectBodies[jointname + "_orientation_w"]] ;
		xo = featureVectorKinectBodies[headerMapKinectBodies[jointname + "_orientation_x"]] ;
		yo = featureVectorKinectBodies[headerMapKinectBodies[jointname + "_orientation_y"]] ;
		zo = featureVectorKinectBodies[headerMapKinectBodies[jointname + "_orientation_z"]] ;

		ros_joint_msg.joint_type = static_cast<uint8_t>( i );
		ros_joint_msg.tracking_state = static_cast<uint8_t>( featureVectorKinectBodies[headerMapKinectBodies[jointname + "_tracking_state"]] );

		ros_joint_msg.position.x = xp;
		ros_joint_msg.position.y = yp;
		ros_joint_msg.position.z = zp;

		ros_joint_msg.orientation.x = xo;
		ros_joint_msg.orientation.y = yo;
		ros_joint_msg.orientation.z = zo;
		ros_joint_msg.orientation.w = wo;

		ros_joints_msg.emplace_back( std::move( ros_joint_msg ) );

	}
	ros_bodies_msg.bodies.emplace_back( std::move( ros_body_msg ) );

	return ros_bodies_msg;
}


_KinectHDFacesMsg GetKinectHDFacesMsg(vector<float> featureVectorKinectHDFaces, map<string, int> headerMapKinectHDFaces)
{
	_KinectHDFacesMsg ros_hdfaces_msg;
	_KinectHDFaceMsg ros_hdface_msg;
	map<KinectHDFaceData::AnimationUnit, string> auLookup = KinectHDFaceData::buildAnimationUnitMap();

	ros_hdface_msg.is_tracked = true;
	ros_hdface_msg.frame = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdface_frame"]];
	ros_hdface_msg.time_stamp = featureVectorKinectHDFaces[headerMapKinectHDFaces["time"]];
	ros_hdface_msg.std_body_idx = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdface_idx"]];

	ros_hdface_msg.tongue_sticking_out = featureVectorKinectHDFaces[headerMapKinectHDFaces["tongue_sticking_out"]];
	ros_hdface_msg.tongue_depth = featureVectorKinectHDFaces[headerMapKinectHDFaces["tongue_depth"]];
	ros_hdface_msg.upper_lip_depth = featureVectorKinectHDFaces[headerMapKinectHDFaces["upperlip_depth"]];
	ros_hdface_msg.face_quality = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdface_quality"]];

	ros_hdface_msg.head_pivot.x = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdhead_pivot_x"]];
	ros_hdface_msg.head_pivot.y = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdhead_pivot_y"]];
	ros_hdface_msg.head_pivot.z = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdhead_pivot_z"]];

	ros_hdface_msg.bounding_box.x = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdfacebox_top"]];
	ros_hdface_msg.bounding_box.y = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdfacebox_bottom"]];
	ros_hdface_msg.bounding_box.z = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdfacebox_left"]];
	ros_hdface_msg.bounding_box.w = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdfacebox_right"]];

	ros_hdface_msg.pose.pitch = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdpitch"]];
	ros_hdface_msg.pose.yaw = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdyaw"]];
	ros_hdface_msg.pose.roll = featureVectorKinectHDFaces[headerMapKinectHDFaces["hdroll"]];

	auto & ros_animationunits_msg = ros_hdface_msg.animation_units;

	for( int i = 0; i < 17; ++i )
	{
	    _KinectAnimationUnitMsg ros_animationunit_msg;
		string auName = auLookup[static_cast<KinectHDFaceData::AnimationUnit>(i)];

	    ros_animationunit_msg.name = auName;
	    ros_animationunit_msg.value = featureVectorKinectHDFaces[headerMapKinectHDFaces[auName]];

	    ros_animationunits_msg.emplace_back( std::move( ros_animationunit_msg ) );
	}

	ros_hdfaces_msg.hdfaces.emplace_back( std::move( ros_hdface_msg ) );

	//kinect_hdfaces_pub_.publish( ros_hdfaces_msg );
	return ros_hdfaces_msg;
}

_KinectFacesMsg GetKinectFacesMsg(vector<float> featureVectorKinectFaces, map<string, int> headerMapKinectFaces)
{
	_KinectFacesMsg ros_faces_msg;
	_KinectFaceMsg ros_face_msg;
	map<KinectFaceData::FaceProperty, string> facePropertyLookup = KinectFaceData::buildFacePropertiesMap();

	ros_face_msg.is_tracked = true;
	ros_face_msg.frame = featureVectorKinectFaces[headerMapKinectFaces["frame"]];
	ros_face_msg.time_stamp = featureVectorKinectFaces[headerMapKinectFaces["time"]];
	ros_face_msg.std_body_idx = featureVectorKinectFaces[headerMapKinectFaces["face_idx"]];


	ros_face_msg.pose.pitch = featureVectorKinectFaces[headerMapKinectFaces["pitch"]];//?
	ros_face_msg.pose.yaw = featureVectorKinectFaces[headerMapKinectFaces["yaw"]];//?
	ros_face_msg.pose.roll = featureVectorKinectFaces[headerMapKinectFaces["roll"]];//?

	auto & ros_faceproperties_msg = ros_face_msg.properties;

	for(int i = 0; i < 8; ++i)
	{
		_KinectFacePropertyMsg ros_faceproperty_msg;
		string faceProperty = facePropertyLookup[static_cast<KinectFaceData::FaceProperty>(i)];
		ros_faceproperty_msg.property_name = faceProperty;
		float facePropertyVal = featureVectorKinectFaces[headerMapKinectFaces[faceProperty]];
		ros_faceproperty_msg.property_value = static_cast<uint8_t>( facePropertyVal );
		ros_faceproperties_msg.emplace_back( std::move( ros_faceproperty_msg ) );
	} 

	ros_faces_msg.faces.emplace_back( std::move( ros_face_msg ) );

	return ros_faces_msg;
}

_CLMHeadsMsg GetCLMHeadMsg(vector<float> featureVectorCLMHeads, map<string, int> headerMapCLMHeads)
{
	map<CLMFacialActionUnitData::AUType, pair<string,string> > auLookup = CLMFacialActionUnitData::buildActionUnitIDsMap();

	_CLMHeadsMsg ros_clmheads_msg;

	_CLMHeadMsg ros_clmhead_msg;

	ros_clmhead_msg.std_body_idx = featureVectorCLMHeads[headerMapCLMHeads["head_idx"]];
	ros_clmhead_msg.detection_success = featureVectorCLMHeads[headerMapCLMHeads["detection_success"]];
	ros_clmhead_msg.detection_confidence = featureVectorCLMHeads[headerMapCLMHeads["detection_confidence"]];
	ros_clmhead_msg.frame = featureVectorCLMHeads[headerMapCLMHeads["frame"]];
	ros_clmhead_msg.time_stamp = featureVectorCLMHeads[headerMapCLMHeads["time"]];

	ros_clmhead_msg.left_eyegaze_cameraref.x = featureVectorCLMHeads[headerMapCLMHeads["left_eyegaze_cameraref_x"]];
	ros_clmhead_msg.left_eyegaze_cameraref.y = featureVectorCLMHeads[headerMapCLMHeads["left_eyegaze_cameraref_y"]];
	ros_clmhead_msg.left_eyegaze_cameraref.z = featureVectorCLMHeads[headerMapCLMHeads["left_eyegaze_cameraref_z"]];
	ros_clmhead_msg.right_eyegaze_cameraref.x = featureVectorCLMHeads[headerMapCLMHeads["right_eyegaze_cameraref_x"]];
	ros_clmhead_msg.right_eyegaze_cameraref.y = featureVectorCLMHeads[headerMapCLMHeads["right_eyegaze_cameraref_y"]];
	ros_clmhead_msg.right_eyegaze_cameraref.z = featureVectorCLMHeads[headerMapCLMHeads["right_eyegaze_cameraref_z"]];
	ros_clmhead_msg.left_eyegaze_headref.x = featureVectorCLMHeads[headerMapCLMHeads["left_eyegaze_headref_x"]];
	ros_clmhead_msg.left_eyegaze_headref.y = featureVectorCLMHeads[headerMapCLMHeads["left_eyegaze_headref_y"]];
	ros_clmhead_msg.left_eyegaze_headref.z = featureVectorCLMHeads[headerMapCLMHeads["left_eyegaze_headref_z"]];
	ros_clmhead_msg.right_eyegaze_headref.x = featureVectorCLMHeads[headerMapCLMHeads["right_eyegaze_headref_x"]];
	ros_clmhead_msg.right_eyegaze_headref.y = featureVectorCLMHeads[headerMapCLMHeads["right_eyegaze_headref_y"]];
	ros_clmhead_msg.right_eyegaze_headref.z =  featureVectorCLMHeads[headerMapCLMHeads["right_eyegaze_headref_z"]];

	ros_clmhead_msg.headpose.pitch = featureVectorCLMHeads[headerMapCLMHeads["headpose_rotation_pitch"]];
	ros_clmhead_msg.headpose.yaw = featureVectorCLMHeads[headerMapCLMHeads["headpose_rotation_yaw"]];
	ros_clmhead_msg.headpose.roll = featureVectorCLMHeads[headerMapCLMHeads["headpose_rotation_roll"]];
	ros_clmhead_msg.headpose.x = featureVectorCLMHeads[headerMapCLMHeads["headpose_translation_x"]];
	ros_clmhead_msg.headpose.y = featureVectorCLMHeads[headerMapCLMHeads["headpose_translation_y"]];
	ros_clmhead_msg.headpose.z = featureVectorCLMHeads[headerMapCLMHeads["headpose_translation_z"]];


	auto & ros_aus_msg = ros_clmhead_msg.aus;


	for(int i = 0; i < 20; ++i)
	{
		_CLMFacialActionUnitMsg ros_au_msg;

		pair< string, string> au_ids = auLookup[static_cast<CLMFacialActionUnitData::AUType>(i)];

		float au_val = featureVectorCLMHeads[headerMapCLMHeads[au_ids.first + "_" + au_ids.second]];

		ros_au_msg.au_id = au_ids.first;
		ros_au_msg.au_value = au_val;
		ros_au_msg.au_prediction_method = au_ids.second;

		ros_aus_msg.emplace_back( std::move( ros_au_msg ) );

	} 

	ros_clmheads_msg.heads.emplace_back( std::move( ros_clmhead_msg ) );
	return ros_clmheads_msg;
}

_KinectRawAudioMsg GetKinectRawAudioMsg(vector<float> featureVectorKinectRawAudio, map<string, int> headerMapKinectRawAudio)
{

	_KinectRawAudioMsg ros_rawaudio_msg;

	ros_rawaudio_msg.time_stamp = featureVectorKinectRawAudio[headerMapKinectRawAudio["time"]];
	ros_rawaudio_msg.frame = featureVectorKinectRawAudio[headerMapKinectRawAudio["frame"]];
	ros_rawaudio_msg.is_tracked = true;
	ros_rawaudio_msg.energy = featureVectorKinectRawAudio[headerMapKinectRawAudio["energy"]];
	
	return ros_rawaudio_msg;
}





  //input: folder, includes files and corresponding personids, cannot assume that person ids are ordered, so how do I order? , assume single person, cojtijuous action, unknwown fps... need to do a glob

  //TODO: add faces, hdfaces,  
  //Write emission alignment function, add booleans to switch certain sources off, validate it works
  //Write publishing function, can use template from kinect client. 
  //resolve time alignment duplication...
  //write person ordering code

  //add volume...
  //what about rate as fast as possible?, what about msgs members for labels.?

  //VALIDATE...
  //MOVE to classification message passing for classfication..

  //VADIM:
  //resolve time alignment duplication, basically add fps 
  //potentially missing hdfaces in data? pls check
  //annotations in the file within a clip?

  //PSEUDOCODE: 
  	// emiiting both as long as + 100ms from the earliest
  	// otherwise emit one 
  	// increment the lowest times up until the second lowest times, or if theres another "low"
  	// if one left, only increment and emit that one


  // 0 | 1 | x2 | 3 | 4 | 5 | 6 | 7 | 8

  // 1 | x1 | 1 | 3 | 5 | 6 | 

  // 1 | x2 | 4 | 5 | 7 | 

  // 0 | 1 | x2 | 3 | 4 | 5 | 6 | 7 | 8

  // 1 | x3 | 5 | 6 | 20 | 23 | 25

  // x2 | 5 | 7 | 

  // 20 |


int main(int argc, char **argv)
{

  this_thread::sleep_for(chrono::milliseconds(6000));
  // Set up ROS.
  //string is name of node
  ros::init(argc, argv, "data_read_node");
  ros::NodeHandle nh;

  int rate = 100000;

  //ros::Publisher pub_message = nh.advertise<multimodal::MultimodalState>("/multimodal/state", 10);

  ros::Rate r(rate);


  ros::Publisher pub_message = nh.advertise<multimodal::MultimodalState>("/multimodal/state", 10);


  // KinectBodyData kbd = nullptr;
  // vector<pair<int,KinectBodyData> > kbd_vect;



  bool processCLMHeads = true;
  bool processKinectBodies = true;
  bool processKinectFaces = true;
  bool processKinectHDFaces = true;
  bool processRawAudio = false;

  //vector<string> labelNames = {"punching_prep","punching","hitting_prep","hitting","pointing"};

  string rootDirectoryPath = "dataInput2";
  vector<string> filenames = GetFileNames(rootDirectoryPath);
  vector<string> filenamesWithPattern = FilenamesContainingPattern(filenames, "experiment_clip");



  for (string folderName : filenamesWithPattern)
  {

  	  string directoryPath = rootDirectoryPath + "/" + folderName;

	  map< string, map<string, int> > headerMap;
	  map< string, vector<vector<float> > > allData;

	  //string directoryPath = "sourceDirectory";
	  vector<string> directoryFilenames = GetFileNames(directoryPath);
	  string labelsFilePath = directoryPath + "/" + FilenamesContainingPattern(directoryFilenames, "labels")[0];
	  map<string, int> labelHeaderMap; //header map
	  
	  //map<int, <vector<string>> labelsFile; //time to vector of labels (including the time) 
	  //void ReadDataFromDirectory(vector<vector<float> >& data, map<string, int>& headermap, string directorypath, vector<string> directoryfilenames, string datasource)

	  unordered_map<float, vector<float> > labelsData;

	  cout << "DIRECTORY PATH " << directoryPath << endl;

	  ReadDataFromDirectoryMap(labelsData, labelHeaderMap, directoryPath, directoryFilenames, "labels"); //works
	  
	  //cout << "Size of labels, (should be huge...) " << labelsData.size() << endl;
	  // return 0;

	  vector<pair<float, string>> ordering = GetAbsTimeSinglePersonIdOrdering(directoryPath,directoryFilenames);

	  //cout << "Size ordering mapping " << ordering.size() << endl;

	  if (processCLMHeads) ReadPersonDataFromDirectory(allData["clmHeads"], headerMap["clmHeads"], directoryPath, directoryFilenames, "CLMHeads", ordering);//"/home/TrainingData/Test/logCLMHeads_4-14-2016_10-41-51_person1.csv");
	  if (processKinectBodies) ReadPersonDataFromDirectory(allData["kinectBodies"], headerMap["kinectBodies"], directoryPath, directoryFilenames, "KinectBodies", ordering);
	  if (processKinectFaces) ReadPersonDataFromDirectory(allData["kinectFaces"], headerMap["kinectFaces"], directoryPath, directoryFilenames, "KinectFaces", ordering);
	  if (processKinectHDFaces) ReadPersonDataFromDirectory(allData["kinectHDFaces"], headerMap["kinectHDFaces"], directoryPath, directoryFilenames, "KinectHDFaces", ordering);
	  if (processRawAudio) ReadDataFromDirectory(allData["kinectRawAudio"], headerMap["kinectRawAudio"], directoryPath, directoryFilenames, "KinectRawAudio");

	  // cout << "Kinect bodies size " << allData["kinectBodies"].size() << endl;
	  // cout << "Kinect faces size " << allData["kinectFaces"].size() << endl;
	  // cout << "Kinect hd faces size " << allData["kinectHDFaces"].size() << endl;
	  // cout << "CLM heads size " << allData["clmHeads"].size() << endl;

	  // cout << "Read feature vectors" << endl;

	  // float prevTime = 0;
	  // for (auto i : allData["kinectBodies"])
	  // {
	  // 	if (i[0] < prevTime )
	  // 	{
	  // 		cout << "ERROR" << endl;
	  // 		break;
	  // 	}
	  // 	cout << "abs time "<< i[0] << endl;
	  // 	prevTime = i[0];
	  // }

	  uint32_t lastMinimumTime = numeric_limits<uint32_t>::max();


	  map<string, int> dataIndices;
	  if (processCLMHeads) dataIndices["clmHeads"] = 0;
	  if (processKinectBodies) dataIndices["kinectBodies"] = 0;
	  if (processKinectFaces) dataIndices["kinectFaces"] = 0;
	  if (processKinectHDFaces) dataIndices["kinectHDFaces"] = 0;
	  if (processRawAudio) dataIndices["kinectRawAudio"] = 0;

	  int numOfSourcesReachedMaxIndex = 0;

	  while (nh.ok() && numOfSourcesReachedMaxIndex != dataIndices.size())
	  {

	  	//get all the data elements at the current indices...

	  	//create a map for featureVectors at a timepoint
	  	map<string, vector<float>> featureVectorMap;

	  	vector<string> minimumKeys; 
	  	uint32_t minimumTime = numeric_limits<uint32_t>::max();

	  	//this extracts features of the data at the minimum indices and finds minimum time 


	  	for (auto const& element : dataIndices)
	  	{

			//get string key in data Indices
			string key = element.first;
			if (dataIndices[key] == allData[key].size()) continue;
			

			featureVectorMap[key] = allData[key][dataIndices[key]];

	  		// find the "indexes" <-> keys that have the lowest times.

			uint32_t featureTime = static_cast<uint32_t>(featureVectorMap[key][0]);

			if (featureTime < minimumTime)
			{
				//creates a new list of keys
				minimumKeys = {key};
				minimumTime = featureTime;
			}
			else if (featureTime == minimumTime)
			{
				//appends to the old one 
				minimumKeys.push_back(key);
			}
	  	}


		//here is where I publish features if <= +100 of minimumTime 

		map<string, bool> sourcesValid;// = map<string, bool>;
		uint32_t alignmentLeniency = 100;

		multimodal::MultimodalState multimodal_msg; 

		multimodal_msg.kinect_bodies_msg_isvalid = false; 
		multimodal_msg.kinect_faces_msg_isvalid = false;
		multimodal_msg.kinect_hdfaces_msg_isvalid = false;
		multimodal_msg.kinect_rawaudio_msg_isvalid = false;
		multimodal_msg.kinect_clmheads_msg_isvalid = false;

	  	for (auto const& element : featureVectorMap)
	  	{
	  		string key = element.first;
	  		vector<float> features = element.second;
	  		if (features[0] <= minimumTime + alignmentLeniency)
	  		{
				if (key == "kinectBodies")  
				{
					multimodal_msg.kinect_bodies_msg_isvalid = true;
					multimodal_msg.kinect_bodies_msg = GetKinectBodyMsg(features, headerMap[key]);
				}
				if (key == "clmHeads") 
				{
					multimodal_msg.kinect_clmheads_msg_isvalid = true;
					multimodal_msg.kinect_clmheads_msg = GetCLMHeadMsg(features, headerMap[key]);
				}	
				if (key == "kinectFaces") 
				{
					multimodal_msg.kinect_faces_msg_isvalid = true;
					multimodal_msg.kinect_faces_msg = GetKinectFacesMsg(features, headerMap[key]);
				}	
				if (key == "kinectHDFaces") 
				{
					multimodal_msg.kinect_hdfaces_msg_isvalid = true;
					multimodal_msg.kinect_hdfaces_msg = GetKinectHDFacesMsg(features, headerMap[key]);
				}		
				if (key == "kinectRawAudio") 
				{
					multimodal_msg.kinect_rawaudio_msg_isvalid = true;
					multimodal_msg.kinect_rawaudio_msg = GetKinectRawAudioMsg(features, headerMap[key]);
				}		
	  		}

		}

		auto & ros_labels_msg = multimodal_msg.labels_msg;
		multimodal_msg.labels_arevalid = false;

		for(auto& header : labelHeaderMap)
		{
			multimodal_msg.labels_arevalid = true;

			_MultimodalLabelMsg ros_label_msg;
			// string faceProperty = facePropertyLookup[static_cast<KinectFaceData::FaceProperty>(i)];
			// ros_faceproperty_msg.property_name = faceProperty;
			// float facePropertyVal = featureVectorKinectFaces[headerMapKinectFaces[faceProperty]];
			// ros_label_msg.property_value = static_cast<uint8_t>( facePropertyVal );
			ros_label_msg.labelName = header.first;
			ros_label_msg.labelValue = labelsData[minimumTime][header.second];
			ros_label_msg.isValid = ros_label_msg.labelValue == -1 ? false : true;


			ros_labels_msg.emplace_back( std::move( ros_label_msg ) );
		} 

		multimodal_msg.timeID = minimumTime; 
		if (abs(minimumTime - lastMinimumTime) > 200) 
		{
			multimodal_msg.new_sequence = true; // difference of 100ms between minTime and LastMinTime
			//cout << "new" << endl;
		}
		else
		{	
			multimodal_msg.new_sequence = false; 
			//cout << "" << endl;
		}

		lastMinimumTime = minimumTime;

		
		if(minimumTime != numeric_limits<uint32_t>::max()) 
		{
			//cout << "publish?" <<endl;
			pub_message.publish(multimodal_msg);
		}
	  	// increment the lowest times, the handle to times is at <>
	  	for (string const& key : minimumKeys)
	  	{
	  		++dataIndices[key]; 
	  		if (dataIndices[key] == allData[key].size()) ++numOfSourcesReachedMaxIndex;
	  	}
	  	// cout << "numMax " << numOfSourcesReachedMaxIndex << endl;
	    ros::spinOnce();
	    r.sleep();
	  }  


	  //return 0;


  }

  return 0;
}
