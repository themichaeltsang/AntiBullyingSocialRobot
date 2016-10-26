// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#include "multimodal/MultimodalState.h"

#include "kinect_bridge2/KinectBodies.h"
#include "kinect_bridge2/KinectFaces.h"
#include "kinect_bridge2/KinectHDFaces.h"
#include "kinect_bridge2/KinectRawAudio.h"
#include "kinect_bridge2/CLMHeads.h"

#include "clm_bridge/ClmHeads.h"

#include <boost/thread/mutex.hpp>


class MonitorMsg
{
	public:
	    MonitorMsg();
	    ~MonitorMsg();

		void publishMessage(ros::Publisher *pub_message, std::pair<kinect_bridge2::KinectBodies,bool> &kinect_bodies_msg, 
			std::pair<kinect_bridge2::KinectFaces,bool> &kinect_faces_msg,
			std::pair<kinect_bridge2::KinectHDFaces,bool> &kinect_hdfaces_msg,
			std::pair<kinect_bridge2::KinectRawAudio,bool> &kinect_rawaudio_msg,
			std::pair<kinect_bridge2::CLMHeads,bool> &kinect_clmheads_msg);

		//void publishMessage(ros::Publisher *pub_message, std::pair<kinect_bridge2::KinectBodies,bool> &kinect_msg, std::pair<clm_bridge::ClmHeads,bool> &clm_msg);

	    void messageCallbackKinectBodies(const kinect_bridge2::KinectBodies &msg);
	   	void messageCallbackKinectFaces(const kinect_bridge2::KinectFaces &msg);
	   	void messageCallbackKinectHDFaces(const kinect_bridge2::KinectHDFaces &msg);
	   	void messageCallbackKinectRawAudio(const kinect_bridge2::KinectRawAudio &msg);
	   	void messageCallbackKinectCLMHeads(const kinect_bridge2::CLMHeads &msg);

	    void messageCallbackClmHeads(const clm_bridge::ClmHeads &msg);

		std::pair<kinect_bridge2::KinectBodies, bool> getKinectBodiesMsg();
		std::pair<kinect_bridge2::KinectFaces, bool> getKinectFacesMsg();
		std::pair<kinect_bridge2::KinectHDFaces, bool> getKinectHDFacesMsg();
		std::pair<kinect_bridge2::KinectRawAudio, bool> getKinectRawAudioMsg();
		std::pair<kinect_bridge2::CLMHeads, bool> getKinectCLMHeadsMsg();

		std::pair<clm_bridge::ClmHeads, bool> getClmHeadsMsg();

	private:
		std::pair<kinect_bridge2::KinectBodies, bool> kinect_bodies_msg_;
		std::pair<kinect_bridge2::KinectFaces, bool> kinect_faces_msg_;
		std::pair<kinect_bridge2::KinectHDFaces, bool> kinect_hdfaces_msg_;
		std::pair<kinect_bridge2::KinectRawAudio, bool> kinect_rawaudio_msg_;
		std::pair<kinect_bridge2::CLMHeads, bool> kinect_clmheads_msg_;

		std::pair<clm_bridge::ClmHeads, bool> clm_msg_;


		boost::mutex bodies_mutex_;
	  	boost::mutex faces_mutex_;
	  	boost::mutex hdfaces_mutex_;
	  	boost::mutex rawaudio_mutex_;
	  	boost::mutex clmheads_mutex_;
    
};