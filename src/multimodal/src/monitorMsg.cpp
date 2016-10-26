#include "multimodal/monitorMsg.h"

MonitorMsg::MonitorMsg()
{

}

MonitorMsg::~MonitorMsg()
{

}


void MonitorMsg::publishMessage
( ros::Publisher *pub_message, 
  std::pair<kinect_bridge2::KinectBodies,bool> &kinect_bodies_msg, 
  std::pair<kinect_bridge2::KinectFaces,bool> &kinect_faces_msg, 
  std::pair<kinect_bridge2::KinectHDFaces,bool> &kinect_hdfaces_msg, 
  std::pair<kinect_bridge2::KinectRawAudio,bool> &kinect_rawaudio_msg, 
  std::pair<kinect_bridge2::CLMHeads,bool> &kinect_clmheads_msg )
{
    multimodal::MultimodalState multimodal_msg; 

    kinect_bridge2::KinectBodies kinectbodies_msg_topublish; //declare variable for msgs to publish
    kinect_bridge2::KinectFaces kinectfaces_msg_topublish;
    kinect_bridge2::KinectHDFaces kinecthdfaces_msg_topublish;
    kinect_bridge2::KinectRawAudio kinectrawaudio_msg_topublish;
    kinect_bridge2::CLMHeads kinectclmheads_msg_topublish;

    kinectbodies_msg_topublish = kinect_bodies_msg.first; //set msg variables to publish 
    kinectfaces_msg_topublish = kinect_faces_msg.first;
    kinecthdfaces_msg_topublish = kinect_hdfaces_msg.first;
    kinectrawaudio_msg_topublish = kinect_rawaudio_msg.first;
    kinectclmheads_msg_topublish = kinect_clmheads_msg.first;

    bool kinectbodiesmsg_valid = kinect_bodies_msg.second ? true : false; //set bool for whether the msg is valid 
    bool kinectfacesmsg_valid = kinect_faces_msg.second ? true : false;
    bool kinecthdfacesmsg_valid = kinect_hdfaces_msg.second ? true : false;
    bool kinectrawaudiomsg_valid = kinect_rawaudio_msg.second ? true : false;
    bool kinectclmheadsmsg_valid = kinect_clmheads_msg.second ? true : false;

    if ( kinectbodiesmsg_valid || kinectfacesmsg_valid || kinecthdfacesmsg_valid || kinectrawaudiomsg_valid || kinectclmheadsmsg_valid )
    {
        //std::cerr << "publishing" << std::endl;
        multimodal_msg.kinect_bodies_msg = kinectbodies_msg_topublish; // write msg topublish
        multimodal_msg.kinect_faces_msg = kinectfaces_msg_topublish;
        multimodal_msg.kinect_hdfaces_msg = kinecthdfaces_msg_topublish;
        multimodal_msg.kinect_rawaudio_msg = kinectrawaudio_msg_topublish;
        multimodal_msg.kinect_clmheads_msg = kinectclmheads_msg_topublish;

        multimodal_msg.kinect_bodies_msg_isvalid = kinectbodiesmsg_valid; // write isvalid bool for corresponding msg
        multimodal_msg.kinect_faces_msg_isvalid = kinectfacesmsg_valid;
        multimodal_msg.kinect_hdfaces_msg_isvalid = kinecthdfacesmsg_valid;
        multimodal_msg.kinect_rawaudio_msg_isvalid = kinectrawaudiomsg_valid;
        multimodal_msg.kinect_clmheads_msg_isvalid = kinectclmheadsmsg_valid;

        pub_message->publish(multimodal_msg);
    }
}


// void MonitorMsg::publishMessage(ros::Publisher *pub_message, std::pair<kinect_bridge2::KinectBodies,bool> &kinect_msg, std::pair<clm_bridge::ClmHeads,bool> &clm_msg)
// {
//     multimodal::MultimodalState multimodal_msg; 

//     kinect_bridge2::KinectBodies kinect_msg_topublish;
//     clm_bridge::ClmHeads clm_msg_topublish;

//     kinect_msg_topublish = kinect_msg.first;
//     clm_msg_topublish = clm_msg.first;

//     bool headsmsg_valid = clm_msg.second ? true : false;
//     bool bodiesmsg_valid = kinect_msg.second ? true : false;


//    // if either msg is valid then publish and include flags for any that isn't valid
//     if ( headsmsg_valid || bodiesmsg_valid )
//     {
//         //std::cerr << "publishing" << std::endl;
//         multimodal_msg.headsMsg = clm_msg_topublish;
//         multimodal_msg.bodiesMsg = kinect_msg_topublish;

//         multimodal_msg.headsMsg_isValid = headsmsg_valid;
//         multimodal_msg.bodiesMsg_isValid = bodiesmsg_valid;

//         pub_message->publish(multimodal_msg);

//     }

// }


void MonitorMsg::messageCallbackKinectBodies(const kinect_bridge2::KinectBodies &msg)
{   
    boost::mutex::scoped_lock lock(bodies_mutex_);
    kinect_bodies_msg_ = std::make_pair(msg,true); //the pair consists of the msg itself and a bool for whether the msg is valid
}

void MonitorMsg::messageCallbackKinectFaces(const kinect_bridge2::KinectFaces &msg)
{
    boost::mutex::scoped_lock lock(faces_mutex_);
    kinect_faces_msg_ = std::make_pair(msg,true); //the pair consists of the msg itself and a bool for whether the msg is valid
}

void MonitorMsg::messageCallbackKinectHDFaces(const kinect_bridge2::KinectHDFaces &msg)
{
    boost::mutex::scoped_lock lock(hdfaces_mutex_);
    kinect_hdfaces_msg_ = std::make_pair(msg,true); //the pair consists of the msg itself and a bool for whether the msg is valid
}

void MonitorMsg::messageCallbackKinectRawAudio(const kinect_bridge2::KinectRawAudio &msg)
{
    boost::mutex::scoped_lock lock(rawaudio_mutex_);
    kinect_rawaudio_msg_ = std::make_pair(msg,true); //the pair consists of the msg itself and a bool for whether the msg is valid
}

void MonitorMsg::messageCallbackKinectCLMHeads(const kinect_bridge2::CLMHeads &msg)
{
    boost::mutex::scoped_lock lock(clmheads_mutex_);
    kinect_clmheads_msg_ = std::make_pair(msg,true); //the pair consists of the msg itself and a bool for whether the msg is valid
}


void MonitorMsg::messageCallbackClmHeads(const clm_bridge::ClmHeads &msg)
{
    clm_msg_ = std::make_pair(msg,true);
}


std::pair<kinect_bridge2::KinectBodies,bool> MonitorMsg::getKinectBodiesMsg()
{    
    boost::mutex::scoped_lock lock(bodies_mutex_);
    return kinect_bodies_msg_;
}
std::pair<kinect_bridge2::KinectFaces,bool> MonitorMsg::getKinectFacesMsg()
{
    boost::mutex::scoped_lock lock(faces_mutex_);
    return kinect_faces_msg_;
}
std::pair<kinect_bridge2::KinectHDFaces,bool> MonitorMsg::getKinectHDFacesMsg()
{
    boost::mutex::scoped_lock lock(hdfaces_mutex_);
    return kinect_hdfaces_msg_;
}
std::pair<kinect_bridge2::KinectRawAudio,bool> MonitorMsg::getKinectRawAudioMsg()
{
    boost::mutex::scoped_lock lock(rawaudio_mutex_);
    return kinect_rawaudio_msg_;
}
std::pair<kinect_bridge2::CLMHeads,bool> MonitorMsg::getKinectCLMHeadsMsg()
{
    boost::mutex::scoped_lock lock(clmheads_mutex_);
    return kinect_clmheads_msg_;
}


std::pair<clm_bridge::ClmHeads,bool> MonitorMsg::getClmHeadsMsg()
{
    return clm_msg_;
}


