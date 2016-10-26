#include "multimodal/monitorMsg.h"
#include "multimodal/MultimodalState.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  //string is name of node
  ros::init(argc, argv, "multimodal");
  ros::NodeHandle nh;

  int rate = 100;

  // Create a new NodeExample object.
  MonitorMsg *kinect_bodies_msg_in = new MonitorMsg();
  MonitorMsg *kinect_rawaudio_msg_in = new MonitorMsg();
  MonitorMsg *kinect_faces_msg_in = new MonitorMsg();
  MonitorMsg *kinect_hdfaces_msg_in = new MonitorMsg();
  MonitorMsg *kinect_clmheads_msg_in = new MonitorMsg();

  //MonitorMsg *clm_msg_in = new MonitorMsg();

  MonitorMsg *multimodal_msg_out = new MonitorMsg();


  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.

  // ros::NodeHandle pnh("~");
  // pnh.param("a", a, 1);
  // pnh.param("b", b, 2);
  // pnh.param("message", message, std::string("hello"));
  // pnh.param("rate", rate, 40);

  // Create a publisher and name the topic.

  ros::Publisher pub_message = nh.advertise<multimodal::MultimodalState>("/multimodal/state", 10);

  // Create a subscriber.
  // Name the topic, msg queue, callback function with class name, and object containing callback function.
  
  ros::Subscriber sub_msg1 = nh.subscribe("/kinect_client/bodies", 1000, &MonitorMsg::messageCallbackKinectBodies, kinect_bodies_msg_in);
  ros::Subscriber sub_msg2 = nh.subscribe("/kinect_client/faces", 1000, &MonitorMsg::messageCallbackKinectFaces, kinect_faces_msg_in);
  ros::Subscriber sub_msg3 = nh.subscribe("/kinect_client/hdfaces", 1000, &MonitorMsg::messageCallbackKinectHDFaces, kinect_hdfaces_msg_in);
  ros::Subscriber sub_msg4 = nh.subscribe("/kinect_client/rawaudio", 1000, &MonitorMsg::messageCallbackKinectRawAudio, kinect_rawaudio_msg_in);
  ros::Subscriber sub_msg5 = nh.subscribe("/kinect_client/clmheads", 1000, &MonitorMsg::messageCallbackKinectCLMHeads, kinect_clmheads_msg_in);


  //ros::Subscriber sub_msg6 = nh.subscribe("/clm_bridge/heads", 1000, &MonitorMsg::messageCallbackClmHeads, clm_msg_in);

  // Tell ROS how fast to run this node.
  ros::Rate r(rate);

  // Main loop.
  while (nh.ok())
  {
  	// std::cout << "kinect_msg_in success?" << sub_msg1 << std::endl;
  	// std::cout << "clm_msg_in success?" << sub_msg2 << std::endl;
    std::pair<kinect_bridge2::KinectBodies,bool> kinect_bodies_msg_out = kinect_bodies_msg_in->getKinectBodiesMsg();
    std::pair<kinect_bridge2::KinectFaces,bool> kinect_faces_msg_out = kinect_faces_msg_in->getKinectFacesMsg();  
    std::pair<kinect_bridge2::KinectHDFaces,bool> kinect_hdfaces_msg_out = kinect_hdfaces_msg_in->getKinectHDFacesMsg();
    std::pair<kinect_bridge2::KinectRawAudio,bool> kinect_rawaudio_msg_out = kinect_rawaudio_msg_in->getKinectRawAudioMsg();
    std::pair<kinect_bridge2::CLMHeads,bool> kinect_clmheads_msg_out = kinect_clmheads_msg_in->getKinectCLMHeadsMsg();

    multimodal_msg_out->publishMessage(&pub_message, kinect_bodies_msg_out, kinect_faces_msg_out, kinect_hdfaces_msg_out, kinect_rawaudio_msg_out, kinect_clmheads_msg_out);

   //std::pair<clm_bridge::ClmHeads,bool> clm_msg_out = clm_msg_in->getClmHeadsMsg();
   //multimodal_msg_out->publishMessage(&pub_message, kinect_bodies_msg_out, clm_msg_out);

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
