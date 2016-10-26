#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>
#include <cstdio>
#include <ctime>

#include <Poco/Net/SocketAddress.h> //should be ok
#include <Poco/Net/StreamSocket.h> //should be ok
#include <Poco/Net/SocketStream.h> //should be ok

#include <atomics/binary_stream.h> //in atomics

#include <messages/kinect_messages.h> //in messages
#include <messages/binary_codec.h> //in messages
#include <messages/message_coder.h> //in messages

#include <messages/png_image_message.h>

#include <messages/input_tcp_device.h> //in messages

#include <ros/ros.h> //should be ok

#include <tf/transform_broadcaster.h> //should be ok

#include <PoseRecognizer.h>

#include <kinect_bridge2/KinectSpeech.h>
#include <kinect_bridge2/KinectBodies.h>
#include <kinect_bridge2/KinectFaces.h>
#include <kinect_bridge2/KinectHDFaces.h>
#include <kinect_bridge2/KinectRawAudio.h>
#include <kinect_bridge2/CLMHeads.h>



#define TRAIN_COUNTDOWN 10
#define INITIAL_TRAIN_COUNTDOWN 1000
#define MAX_TRAINING_SAMPLES 500
//in ms * ~4

class KinectBridge2Client
{
public:
    typedef kinect_bridge2::KinectSpeech _KinectSpeechMsg;
    typedef kinect_bridge2::KinectSpeechPhrase _KinectSpeechPhraseMsg;

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

    std::string training_pose_name_;
    std::vector<std::string> testing_pose_names_;
    PoseRecognition::PoseRecognizer pose_recognizer_;
    bool train_now_;
    static const bool train_ = false;
    int train_countdown_;
    bool take_snapshot_;
    bool first_time_training_;
    int training_samples_count_;

    clock_t timer_start_;

    ros::NodeHandle nh_rel_;
    ros::Publisher kinect_speech_pub_;
    ros::Publisher kinect_bodies_pub_;
    ros::Publisher kinect_faces_pub_;
	ros::Publisher kinect_hdfaces_pub_;
	ros::Publisher kinect_rawaudio_pub_;
	ros::Publisher clm_heads_pub_;

    InputTCPDevice kinect_bridge_client_;

    uint32_t message_count_;

    MessageCoder<BinaryCodec<> > binary_message_coder_;

    tf::TransformBroadcaster transform_broadcaster_;

    KinectBridge2Client( ros::NodeHandle & nh_rel )
    :
        nh_rel_( nh_rel ),
        kinect_speech_pub_( nh_rel_.advertise<_KinectSpeechMsg>( "speech", 10 ) ),
        kinect_bodies_pub_( nh_rel_.advertise<_KinectBodiesMsg>( "bodies", 10 ) ),
        kinect_faces_pub_( nh_rel_.advertise<_KinectFacesMsg>( "faces", 10 ) ),
        kinect_hdfaces_pub_( nh_rel_.advertise<_KinectHDFacesMsg>( "hdfaces", 10 ) ),
        kinect_rawaudio_pub_( nh_rel_.advertise<_KinectRawAudioMsg>( "rawaudio", 10 ) ),
        clm_heads_pub_( nh_rel_.advertise<_CLMHeadsMsg>( "clmheads", 10 ) ),
        kinect_bridge_client_( getParam<std::string>( nh_rel_, "server_ip", "localhost" ), getParam<int>( nh_rel_, "server_port", 5903 ) ),
        message_count_( 0 ),
        train_now_( false ),
        train_countdown_(INITIAL_TRAIN_COUNTDOWN),
        training_samples_count_( 0 ),
        timer_start_(clock()),
        testing_pose_names_( {"crossed_arms","hands_at_ears","left_arm_pointing","right_arm_pointing","left_hand_covering_mouth","right_hand_covering_mouth","shove_preparation","left_hit_preparation","right_hit_preparation","left_kick_preparation","right_kick_preparation","left_arm_swing_preparation","right_arm_swing_preparation","showing_left_fist","showing_right_fist"} ), //right hand hit doesnt exist...
        training_pose_name_( "hands_at_ears" ),//( "shove" ), //not good
        pose_recognizer_(train_, timer_start_, testing_pose_names_, training_pose_name_),
        take_snapshot_( true )

    {        //
    }

    template<class __Data>
    static __Data getParam( ros::NodeHandle & nh, std::string const & param_name, __Data const & default_value )
    {
        __Data result;
        if( nh.getParam( param_name, result ) ) return result;
        return default_value;
    }

    //returns body orientation in the craniocaudal (axial) axis, 
    //where oriented clockwise yields a positive angle < 90 degrees, 
    //oriented counterclockwise yields negative angle > -90 degrees
    //the function accomodates for when the back of the body is turned towards you
    // x1/x2 = left/right corresponding body joints (e.g. shoulder) in coronal(frontal) plane
    // z1/z2 = left/right corresponding body joints (e.g. shoulder) in transverse plane
    static double getRotationAngle(double x1, double x2, double z1, double z2)
    {
        return double( std::atan2((z2 - z1),(x2 - x1)) * (180/M_PI) );
        //return angle;
        // if (x1 == x2)
        // {
        //     if (z1 < z2) return double(90);
        //     else return double(-90);
        // }
        // else
        // {
        //     double angle;
        //     angle = std::atan((z2 - z1)/(x2 - x1)) * (180/M_PI);
        //     if (x1 > x2)
        //     {
        //         if (z1 < z2) return double(180 - angle);
        //         else return double(angle - 180);
        //     }
        //     else return angle;
        // }
    }

    void spin()
    {
        auto last_update = std::chrono::high_resolution_clock::now();
        auto last_update2 = std::chrono::high_resolution_clock::now();

        //int train_countdown = 5; 
        std::cout << "reached this point" << std::endl;
        while( ros::ok() )
        {
            auto now = std::chrono::high_resolution_clock::now();

            if( (std::chrono::duration_cast<std::chrono::milliseconds>( now - last_update ).count() >= 1) && (training_samples_count_ < MAX_TRAINING_SAMPLES) )
            {
                last_update = now;

                if (train_ && train_countdown_ > 0) {
                    std::cout << "countdown to training: " << train_countdown_ << std::endl;
                    train_countdown_--;
                }
                else if (train_ && train_countdown_ == 0 ) { 
                    std::cout << "~~ training now ~~" << std::endl;
                    train_now_ = true;
                    train_countdown_--;
                    training_samples_count_++;

                }
                // else {
                //     std::cout << "processed " << message_count_ << " messages" << std::endl;
                // }
                
            }
            
            if( std::chrono::duration_cast<std::chrono::milliseconds>( now - last_update2 ).count() >= 1000 )
            {
                last_update2 = now;
                //std::cout << "processed " << message_count_ << " messages" << std::endl;
            }


            try
            {
                if( !kinect_bridge_client_.input_socket_.impl()->initialized() )
                {
                    std::cout << "no server connection" << std::endl;
                    std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
                    kinect_bridge_client_.openInput();
                    continue;
                }

                CodedMessage<> binary_coded_message;
                kinect_bridge_client_.pull( binary_coded_message );
                processKinectMessage( binary_coded_message );
                message_count_ ++;
                //std::cout << "message processed" << std::endl;
            }
            catch( messages::MessageException & e )
            {
                std::cout << e.what() << std::endl;
                std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
            }
            catch( std::exception & e )
            {
                std::cout << e.what() << std::endl;
                std::this_thread::sleep_for( std::chrono::milliseconds( 100 ) );
            }

            ros::spinOnce();
        }
    }

    void processKinectMessage( CodedMessage<> & coded_message )
    {
        auto & coded_header = coded_message.header_;
//        std::cout << "processing message type: " << coded_message.header_.payload_type_ << std::endl;

        if( coded_header.payload_id_ == KinectSpeechMessage::ID() )
        {
            auto kinect_speech_message = binary_message_coder_.decode<KinectSpeechMessage>( coded_message );

            auto & header = kinect_speech_message.header_;
            auto & payload = kinect_speech_message.payload_;

            _KinectSpeechMsg ros_kinect_speech_message;

            for( size_t i = 0; i < payload.size(); ++i )
            {
                _KinectSpeechPhraseMsg ros_kinect_speech_phrase_message;
                ros_kinect_speech_phrase_message.tag = payload[i].tag_;
                ros_kinect_speech_phrase_message.confidence = payload[i].confidence_;
                ros_kinect_speech_message.phrases.emplace_back( std::move( ros_kinect_speech_phrase_message ) );
            }

            kinect_speech_pub_.publish( ros_kinect_speech_message );
        }



        else if( coded_header.payload_id_ == KinectColorImageMessage<PNGImageMessage<> >::ID() )
        {
            auto color_image_msg = binary_message_coder_.decode<KinectColorImageMessage<PNGImageMessage<> > >( coded_message );

            auto const & header = color_image_msg.header_;
            auto const & payload = color_image_msg.payload_;

            std::cout << "image height: " << header.height_ << " image width: " << header.width_ << std::endl;

            if (take_snapshot_) {
	            std::ofstream output_stream( "test_kinect_RGB.png", std::ios::binary );
	            color_image_msg.packAs<PNGImageMessage<>>(output_stream);
	            output_stream.close();
	            // take_snapshot = false;
	        }
        }


        else if( coded_header.payload_id_ == KinectFacesMessage::ID() )
        {
            auto faces_msg = binary_message_coder_.decode<KinectFacesMessage>( coded_message );

            auto const & header = faces_msg.header_;
            auto const & payload = faces_msg.payload_;

            _KinectFacesMsg ros_faces_msg;

           	for( size_t face_idx = 0; face_idx < payload.size(); ++face_idx )
            {
                _KinectFaceMsg ros_face_msg;

                auto const & face_msg = payload[face_idx];

                ros_face_msg.is_tracked = face_msg.is_tracked_;
				ros_face_msg.frame = face_msg.frame_;
				ros_face_msg.time_stamp = face_msg.time_stamp_;
                ros_face_msg.std_body_idx = face_msg.std_body_idx_;

                ros_face_msg.pose.pitch = face_msg.headpose_rotation_.pitch;//?
                ros_face_msg.pose.yaw = face_msg.headpose_rotation_.yaw;//?
                ros_face_msg.pose.roll = face_msg.headpose_rotation_.roll;//?

                auto const & faceproperties_msg = face_msg.properties_;
                auto & ros_faceproperties_msg = ros_face_msg.properties;

                // for each joint message
                for( size_t property_idx = 0; property_idx < faceproperties_msg.size(); ++property_idx )
                {
                    auto const & faceproperty_msg = faceproperties_msg[property_idx];
                    _KinectFacePropertyMsg ros_faceproperty_msg;


                    ros_faceproperty_msg.property_name = faceproperty_msg.property_name_;
                    ros_faceproperty_msg.property_value = static_cast<uint8_t>( faceproperty_msg.property_value_ );


                    ros_faceproperties_msg.emplace_back( std::move( ros_faceproperty_msg ) );
                }

                ros_faces_msg.faces.emplace_back( std::move( ros_face_msg ) );
            }

            kinect_faces_pub_.publish( ros_faces_msg );
        }


     	else if( coded_header.payload_id_ == KinectHDFacesMessage::ID() )
        {
            auto hdfaces_msg = binary_message_coder_.decode<KinectHDFacesMessage>( coded_message );

            auto const & header = hdfaces_msg.header_;
            auto const & payload = hdfaces_msg.payload_;

           _KinectHDFacesMsg ros_hdfaces_msg;

           	for( size_t hdface_idx = 0; hdface_idx < payload.size(); ++hdface_idx )
            {
                _KinectHDFaceMsg ros_hdface_msg;

                auto const & hdface_msg = payload[hdface_idx];

                ros_hdface_msg.is_tracked = hdface_msg.is_tracked_;
				ros_hdface_msg.frame = hdface_msg.frame_;
				ros_hdface_msg.time_stamp = hdface_msg.time_stamp_;
                ros_hdface_msg.std_body_idx = hdface_msg.std_body_idx_;

                ros_hdface_msg.tongue_sticking_out = hdface_msg.tongue_sticking_out_;
                ros_hdface_msg.tongue_depth = hdface_msg.tongue_depth_;
                ros_hdface_msg.upper_lip_depth = hdface_msg.upper_lip_depth_;
                ros_hdface_msg.face_quality = hdface_msg.face_quality_;

                ros_hdface_msg.head_pivot.x = hdface_msg.head_pivot_.x;
                ros_hdface_msg.head_pivot.y = hdface_msg.head_pivot_.y;
                ros_hdface_msg.head_pivot.z = hdface_msg.head_pivot_.z;

                ros_hdface_msg.bounding_box.x = hdface_msg.bounding_box_.x;
                ros_hdface_msg.bounding_box.y = hdface_msg.bounding_box_.y;
                ros_hdface_msg.bounding_box.z = hdface_msg.bounding_box_.z;
                ros_hdface_msg.bounding_box.w = hdface_msg.bounding_box_.w;

                ros_hdface_msg.pose.pitch = hdface_msg.headpose_rotation_.pitch;//?
                ros_hdface_msg.pose.yaw = hdface_msg.headpose_rotation_.yaw;//?
                ros_hdface_msg.pose.roll = hdface_msg.headpose_rotation_.roll;//?


                auto const & animationunits_msg = hdface_msg.animation_units_;
                auto & ros_animationunits_msg = ros_hdface_msg.animation_units;


                for( size_t au_idx = 0; au_idx < animationunits_msg.size(); ++au_idx )
                {
                    auto const & animationunit_msg = animationunits_msg[au_idx];
                    _KinectAnimationUnitMsg ros_animationunit_msg;


                    ros_animationunit_msg.name = animationunit_msg.name_;
                    ros_animationunit_msg.value = animationunit_msg.value_;


                    ros_animationunits_msg.emplace_back( std::move( ros_animationunit_msg ) );
                }

                ros_hdfaces_msg.hdfaces.emplace_back( std::move( ros_hdface_msg ) );
            }

            kinect_hdfaces_pub_.publish( ros_hdfaces_msg );
        }


        else if( coded_header.payload_id_ == KinectRawAudioMessage::ID() ) // this only gives me volume data...
        {
            auto rawaudio_msg = binary_message_coder_.decode<KinectRawAudioMessage>( coded_message );
           _KinectRawAudioMsg ros_rawaudio_msg;

			ros_rawaudio_msg.time_stamp = rawaudio_msg.time_stamp_;
			ros_rawaudio_msg.frame = rawaudio_msg.frame_;
           	ros_rawaudio_msg.is_tracked = rawaudio_msg.is_tracked_;
			ros_rawaudio_msg.energy = rawaudio_msg.energy_;
			kinect_rawaudio_pub_.publish( ros_rawaudio_msg );

        }


        else if( coded_header.payload_id_ == KinectBodiesMessage::ID() )
        {
            auto bodies_msg = binary_message_coder_.decode<KinectBodiesMessage>( coded_message );

            auto const & header = bodies_msg.header_;
            auto const & payload = bodies_msg.payload_;

            _KinectBodiesMsg ros_bodies_msg;

            // get map of KinectJointMessage::JointType -> human-readable name
            auto const & joint_names_map = KinectJointMessage::getJointNamesMap();

            // for each body message
            for( size_t body_idx = 0; body_idx < payload.size(); ++body_idx )
            {
                _KinectBodyMsg ros_body_msg;

                auto const & body_msg = payload[body_idx];

                ros_body_msg.is_tracked = body_msg.is_tracked_;
                ros_body_msg.std_body_idx = body_msg.std_body_idx_;

                ros_body_msg.frame = body_msg.frame_; // frame
                ros_body_msg.time_stamp = body_msg.time_stamp_; //include timestamp

                ros_body_msg.hand_state_left = static_cast<uint8_t>( body_msg.hand_state_left_ );
                ros_body_msg.hand_state_right = static_cast<uint8_t>( body_msg.hand_state_right_ );

                auto const & joints_msg = body_msg.joints_;
                auto & ros_joints_msg = ros_body_msg.joints;

				ros_body_msg.lean.x = static_cast<float>(body_msg.lean_.x);
				ros_body_msg.lean.y = static_cast<float>(body_msg.lean_.y);

				// if (ros_body_msg.is_tracked)
				//     std::cout << "leanX: " << ros_body_msg.lean.x << " leanY: " << ros_body_msg.lean.y << std::endl;

                std::stringstream tf_frame_basename_ss;
                tf_frame_basename_ss << "/kinect_client/skeleton" << body_idx << "/";
                //vector armpose_predictions
                std::map<std::string, tf::Transform> joint_transforms_map;
                std::map<std::string, std::pair<double, double>> body_twists_features;
                std::map<std::string, tf::Vector3> joint_positions;
                // std::map<std::string, double> body_twists;

                // for each joint message
                for( size_t joint_idx = 0; joint_idx < joints_msg.size(); ++joint_idx )
                {
                    auto const & joint_msg = joints_msg[joint_idx];
                    _KinectJointMsg ros_joint_msg;

                    ros_joint_msg.joint_type = static_cast<uint8_t>( joint_msg.joint_type_ );
                    ros_joint_msg.tracking_state = static_cast<uint8_t>( joint_msg.tracking_state_ );

                    ros_joint_msg.position.x = joint_msg.position_.x;
                    ros_joint_msg.position.y = joint_msg.position_.y;
                    ros_joint_msg.position.z = joint_msg.position_.z;

                    ros_joint_msg.orientation.x = joint_msg.orientation_.x;
                    ros_joint_msg.orientation.y = joint_msg.orientation_.y;
                    ros_joint_msg.orientation.z = joint_msg.orientation_.z;
                    ros_joint_msg.orientation.w = joint_msg.orientation_.w;

                    //std::cout << "JX: " << ros_joint_msg.orientation.x << std::endl;

                    ros_joints_msg.emplace_back( std::move( ros_joint_msg ) );

                    tf::Transform joint_transform
                    (
                        joint_msg.tracking_state_ == KinectJointMessage::TrackingState::TRACKED ? tf::Quaternion( joint_msg.orientation_.x, joint_msg.orientation_.y, joint_msg.orientation_.z, joint_msg.orientation_.w ).normalized() : tf::Quaternion( 0, 0, 0, 1 ),
                        tf::Vector3( joint_msg.position_.x, joint_msg.position_.y, joint_msg.position_.z )
                    );

                    // if the rotation is nan, zero it out to make TF happy
                    if( std::isnan( joint_transform.getRotation().getAngle() ) ) joint_transform.setRotation( tf::Quaternion( 0, 0, 0, 1 ) );

                    static tf::Transform const trunk_norm_rotation_tf( tf::Quaternion( -M_PI_2, -M_PI_2, 0 ).normalized() );

                    switch( joint_msg.joint_type_ )
                    {
                    case KinectJointMessage::JointType::SPINE_BASE:
                    case KinectJointMessage::JointType::SPINE_MID:
                    case KinectJointMessage::JointType::NECK:
                    case KinectJointMessage::JointType::HEAD:
                    case KinectJointMessage::JointType::SPINE_SHOULDER:
                        joint_transform *= trunk_norm_rotation_tf;
                        break;
                    default:
                        break;
                    }
                    std::string joint_name = joint_names_map.find(joint_msg.joint_type_)->second;
                    transform_broadcaster_.sendTransform( tf::StampedTransform( joint_transform, ros::Time::now(), "/kinect", tf_frame_basename_ss.str() + joint_name ) );
                    joint_transforms_map[joint_name] = joint_transform;

                    if (joint_name == "hip_left" || joint_name == "hip_right" || joint_name == "foot_left" || joint_name == "foot_right" || joint_name == "spine_mid")
                    {
                        body_twists_features[joint_name] = std::make_pair(ros_joint_msg.position.x, ros_joint_msg.position.z);
                    }
                    if (joint_name == "head" || joint_name == "spine_mid")
                    {
                        joint_positions[joint_name] = tf::Vector3(ros_joint_msg.position.x, ros_joint_msg.position.y, ros_joint_msg.position.z);
                    }

                }


                double hip_twist_angle =  getRotationAngle(body_twists_features["hip_left"].first, body_twists_features["hip_right"].first, body_twists_features["hip_left"].second, body_twists_features["hip_right"].second);
                //double shoulder_stance_twist_angle = getRotationAngle(body_twists_features["shoulder_left"].first, body_twists_features["shoulder_right"].first, body_twists_features["shoulder_left"].second, body_twists_features["shoulder_right"].second);
                double foot_stance_twist_angle =  getRotationAngle(body_twists_features["foot_left"].first, body_twists_features["foot_right"].first, body_twists_features["foot_left"].second, body_twists_features["foot_right"].second);
                double proxemics = body_twists_features["spine_mid"].second;

                // body_twists["hip_twist"] = hip_twist_angle;
                //body_twists["shoulder_stance_twist"] = shoulder_stance_twist_angle;
                //body_twists["foot_stance_twist"] = foot_stance_twist_angle;
                
                // if (body_msg.is_tracked_)
                // {
                //     after some tests hip_stance_twist is basically the same as shoulder_stance_twist
                //     std::cout << "hip_twist: " << hip_twist_angle  << "; foot_stance_twist: " << foot_stance_twist_angle << "; shoulder_stance_twist: " << shoulder_stance_twist_angle << std::endl;
                //     std::cout << "proxemics: " << proxemics << " meters; hip_twist: " << hip_twist_angle  << "; foot_stance_twist: " << foot_stance_twist_angle << std::endl;
                // }

                std::map<std::string, double> pose_predictions;

                if (train_ && train_now_)
                {
                    if ( !body_msg.is_tracked_)
                    {
                        std::cout<< "Body isn't tracked; stalling training"<< std::endl;
                        train_countdown_ = TRAIN_COUNTDOWN;
                    }
                    else 
                    {
                        pose_recognizer_.TrainReferencePose(joint_transforms_map);
                    }
                }
                else if (!train_) 
                {
                    pose_predictions = pose_recognizer_.PredictCurrentPoses(joint_transforms_map);
                    // //test
                    if ( body_msg.is_tracked_)
                    {
                        // Testing Thresholds



                        // if (pose_predictions["right_arm_swing_preparation"] < 1.8){
                        //            std::cout << "right_arm_swing_preparation: " << pose_predictions["right_arm_swing_preparation"] << std::endl;
                        // }
                                         

                        // if (pose_predictions["left_arm_swing_preparation"] < 1.8){
                        //           std::cout << "left_arm_swing_preparation: " << pose_predictions["left_arm_swing_preparation"] << std::endl;
                        // }

                        // if (pose_predictions["right_kick_preparation"] > 39){
                        //          std::cout << "right_kick_preparation: " << pose_predictions["right_kick_preparation"] << std::endl;
                        // }

                        // if (pose_predictions["left_kick_preparation"] > 42){
                        //         std::cout << "left_kick_preparation: " << pose_predictions["left_kick_preparation"] << std::endl;
                        // }

                        // if (pose_predictions["right_hit_preparation"] < 2){
                        //       std::cout << "right_hit_preparation: " << pose_predictions["right_hit_preparation"] << std::endl;
                        // }

                        // if (pose_predictions["left_hit_preparation"] < 1.3){
                        //      std::cout << "left_hit_preparation: " << pose_predictions["left_hit_preparation"] << std::endl;
                        // }

                        // if (pose_predictions["shove_preparation"] < 2){
                        //      std::cout << "shove_preparation: " << pose_predictions["shove_preparation"] << std::endl;
                        // }

                        // if (pose_predictions["right_hand_covering_mouth"] < 1.5){
                        //     std::cout << "right_hand_covering_mouth: " << pose_predictions["right_hand_covering_mouth"] << std::endl;
                        // }


                       // if (pose_predictions["left_hand_covering_mouth"] < 1.4){
                       //       std::cout << "left_hand_covering_mouth: " << pose_predictions["left_hand_covering_mouth"] << std::endl;
                       //  }

 
                        // if (pose_predictions["showing_left_fist"] < 0.9){ //note body orientation... //WHY GREATER THAN??
                        //       std::cout << "showing_left_fist: " << pose_predictions["showing_left_fist"] << std::endl;   
                        // }                


                        // if (pose_predictions["showing_right_fist"] < 0.9){ //note body orientation...
                        //      std::cout << "showing_right_fist: " << pose_predictions["showing_right_fist"] << std::endl;   
                        // }                

                        // needs lots of variation on pose
                        // if (pose_predictions["hands_at_ears"] > 42){ //note body orientation...

                        //        std::cout << "hands_at_ears: " << pose_predictions["hands_at_ears"] << std::endl;   
                        // }    

    
                        // if (pose_predictions["right_arm_pointing"] < 1.5){
                        //     std::cout << "right_arm_pointing: " << pose_predictions["right_arm_pointing"] << std::endl;
                        // }

                        // if (pose_predictions["left_arm_pointing"] < 2.5){
                        //       std::cout << "left_arm_pointing: " << pose_predictions["left_arm_pointing"] << std::endl;
                        // }

    
                    }
                }

                ros_body_msg.orientation.hip_twist = static_cast<float>(hip_twist_angle);
                ros_body_msg.orientation.foot_stance_twist = static_cast<float>(foot_stance_twist_angle);
                ros_body_msg.proxemics = static_cast<float>(proxemics);
                ros_body_msg.head_position.x = joint_positions["head"].x();
                ros_body_msg.head_position.y = joint_positions["head"].y();
                ros_body_msg.head_position.z = joint_positions["head"].z();

                ros_body_msg.spine_mid_position.x = joint_positions["spine_mid"].x();
                ros_body_msg.spine_mid_position.y = joint_positions["spine_mid"].y();
                ros_body_msg.spine_mid_position.z = joint_positions["spine_mid"].z();

                //ros_body_msg.pose.crossed_arms = static_cast<float>(pose_predictions["crossed_arms"]);

                ros_body_msg.pose.shoulders_up_closed_stance = static_cast<float>(pose_predictions["shoulders_up_closed_stance"]);
                ros_body_msg.pose.left_arm_pointing = static_cast<float>(pose_predictions["left_arm_pointing"]);
                ros_body_msg.pose.right_arm_pointing = static_cast<float>(pose_predictions["right_arm_pointing"]);
                ros_body_msg.pose.left_hand_covering_mouth = static_cast<float>(pose_predictions["left_hand_covering_mouth"]);
                ros_body_msg.pose.right_hand_covering_mouth = static_cast<float>(pose_predictions["right_hand_covering_mouth"]);
                ros_body_msg.pose.hands_at_ears = static_cast<float>(pose_predictions["hands_at_ears"]);
                ros_body_msg.pose.showing_right_fist = static_cast<float>(pose_predictions["showing_right_fist"]);
                ros_body_msg.pose.showing_left_fist = static_cast<float>(pose_predictions["showing_left_fist"]);
                ros_body_msg.pose.shove_preparation = static_cast<float>(pose_predictions["shove_preparation"]);
                ros_body_msg.pose.right_hit_preparation = static_cast<float>(pose_predictions["right_hit_preparation"]);
                ros_body_msg.pose.left_hit_preparation = static_cast<float>(pose_predictions["left_hit_preparation"]);
                ros_body_msg.pose.right_kick_preparation = static_cast<float>(pose_predictions["right_kick_preparation"]);
                ros_body_msg.pose.left_kick_preparation = static_cast<float>(pose_predictions["left_kick_preparation"]);
                ros_body_msg.pose.right_swing_preparation = static_cast<float>(pose_predictions["right_arm_swing_preparation"]);
                ros_body_msg.pose.left_swing_preparation = static_cast<float>(pose_predictions["left_arm_swing_preparation"]);

                // ros_body_msg.pose.showing_right_fist_elbow_up = static_cast<float>(pose_predictions["showing_right_fist_elbow_up"]);
                // ros_body_msg.pose.showing_left_fist_elbow_up = static_cast<float>(pose_predictions["showing_left_fist_elbow_up"]);
                // ros_body_msg.pose.showing_right_fist_elbow_down = static_cast<float>(pose_predictions["showing_right_fist_elbow_down"]);
                // ros_body_msg.pose.showing_left_fist_elbow_down = static_cast<float>(pose_predictions["showing_left_fist_elbow_down"]);
                // ros_body_msg.pose.shove = static_cast<float>(pose_predictions["shove"]);
                // ros_body_msg.pose.right_hit = static_cast<float>(pose_predictions["right_hit"]);
                // ros_body_msg.pose.left_hit = static_cast<float>(pose_predictions["left_hit"]);
                // ros_body_msg.pose.right_kick = static_cast<float>(pose_predictions["right_kick"]);
                // ros_body_msg.pose.left_kick = static_cast<float>(pose_predictions["left_kick"]);
                // ros_body_msg.pose.right_arm_vertical_swing = static_cast<float>(pose_predictions["right_arm_vertical_swing"]);
                // ros_body_msg.pose.left_arm_vertical_swing = static_cast<float>(pose_predictions["left_arm_vertical_swing"]);
                // ros_body_msg.pose.right_arm_horizontal_swing = static_cast<float>(pose_predictions["right_arm_horizontal_swing"]);
                // ros_body_msg.pose.left_arm_horizontal_swing = static_cast<float>(pose_predictions["left_arm_horizontal_swing"]);
                // ros_body_msg.pose.right_arm_vertical_swing_preparation = static_cast<float>(pose_predictions["right_arm_vertical_swing_preparation"]);
                // ros_body_msg.pose.left_arm_vertical_swing_preparation = static_cast<float>(pose_predictions["left_arm_vertical_swing_preparation"]);
                // ros_body_msg.pose.right_arm_horizontal_swing_preparation = static_cast<float>(pose_predictions["right_arm_horizontal_swing_preparation"]);
                // ros_body_msg.pose.left_arm_horizontal_swing_preparation = static_cast<float>(pose_predictions["left_arm_horizontal_swing_preparation"]);
                // ros_body_msg.pose.right_arm_diagonal_swing_preparation = static_cast<float>(pose_predictions["right_arm_diagonal_swing_preparation"]);
                // ros_body_msg.pose.left_arm_diagonal_swing_preparation = static_cast<float>(pose_predictions["left_arm_diagonal_swing_preparation"]);

                ros_bodies_msg.bodies.emplace_back( std::move( ros_body_msg ) );
            }
            if (train_ && train_now_)
            {

                train_now_ = false;
                train_countdown_ = TRAIN_COUNTDOWN;

            }
            kinect_bodies_pub_.publish( ros_bodies_msg );
        }


        else if( coded_header.payload_id_ == CLMHeadsMessage::ID() )
        {
        	auto clmheads_msg = binary_message_coder_.decode<CLMHeadsMessage>( coded_message );

        	auto const & header = clmheads_msg.header_;
        	auto const & payload = clmheads_msg.payload_;


            _CLMHeadsMsg ros_clmheads_msg;

           	for( size_t head_idx = 0; head_idx < payload.size(); ++head_idx )
            {
                _CLMHeadMsg ros_clmhead_msg;

                auto const & clmhead_msg = payload[head_idx];

				// if (clmhead_msg.detection_success_)
				// {
				// 	std::cout << "pitch: " << clmhead_msg.headpose_rotation_.pitch << " yaw: " << clmhead_msg.headpose_rotation_.yaw << " roll: " << clmhead_msg.headpose_rotation_.roll << std::endl;
				// }

                ros_clmhead_msg.std_body_idx = clmhead_msg.std_body_idx_;
                ros_clmhead_msg.detection_success = clmhead_msg.detection_success_;
				ros_clmhead_msg.detection_confidence = clmhead_msg.detection_confidence_;
				ros_clmhead_msg.frame = clmhead_msg.frame_;
				ros_clmhead_msg.time_stamp = clmhead_msg.time_stamp_;

				ros_clmhead_msg.left_eyegaze_cameraref.x = clmhead_msg.left_eyegaze_cameraref_.x;
				ros_clmhead_msg.left_eyegaze_cameraref.y = clmhead_msg.left_eyegaze_cameraref_.y;
				ros_clmhead_msg.left_eyegaze_cameraref.z = clmhead_msg.left_eyegaze_cameraref_.z;
				ros_clmhead_msg.right_eyegaze_cameraref.x = clmhead_msg.right_eyegaze_cameraref_.x;
				ros_clmhead_msg.right_eyegaze_cameraref.y = clmhead_msg.right_eyegaze_cameraref_.y;
				ros_clmhead_msg.right_eyegaze_cameraref.z = clmhead_msg.right_eyegaze_cameraref_.z;
				ros_clmhead_msg.left_eyegaze_headref.x = clmhead_msg.left_eyegaze_headref_.x;
				ros_clmhead_msg.left_eyegaze_headref.y = clmhead_msg.left_eyegaze_headref_.y;
				ros_clmhead_msg.left_eyegaze_headref.z = clmhead_msg.left_eyegaze_headref_.z;
				ros_clmhead_msg.right_eyegaze_headref.x = clmhead_msg.right_eyegaze_headref_.x;
				ros_clmhead_msg.right_eyegaze_headref.y = clmhead_msg.right_eyegaze_headref_.y;
				ros_clmhead_msg.right_eyegaze_headref.z = clmhead_msg.right_eyegaze_headref_.z;

				ros_clmhead_msg.headpose.pitch = clmhead_msg.headpose_rotation_.pitch;
				ros_clmhead_msg.headpose.yaw = clmhead_msg.headpose_rotation_.yaw;
				ros_clmhead_msg.headpose.roll = clmhead_msg.headpose_rotation_.roll;
				ros_clmhead_msg.headpose.x = clmhead_msg.headpose_translation_.x;
				ros_clmhead_msg.headpose.y = clmhead_msg.headpose_translation_.y;
				ros_clmhead_msg.headpose.z = clmhead_msg.headpose_translation_.z;

 
                auto const & aus_msg = clmhead_msg.facs_;
                auto & ros_aus_msg = ros_clmhead_msg.aus;

                // for each joint message
                for( size_t au_idx = 0; au_idx < aus_msg.size(); ++au_idx )
                {
                    auto const & au_msg = aus_msg[au_idx];
                    _CLMFacialActionUnitMsg ros_au_msg;

                    ros_au_msg.au_id = au_msg.au_id_;
                    ros_au_msg.au_value = au_msg.au_value_;
                    ros_au_msg.au_prediction_method = au_msg.au_prediction_method_;

                    ros_aus_msg.emplace_back( std::move( ros_au_msg ) );
                }

                ros_clmheads_msg.heads.emplace_back( std::move( ros_clmhead_msg ) );
            }

            clm_heads_pub_.publish( ros_clmheads_msg );

        }
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "kinect_client" );
    ros::NodeHandle nh_rel( "~" );

    KinectBridge2Client kinect_bridge2_client( nh_rel );

    std::cout << "before spinning" << std::endl;
    kinect_bridge2_client.spin();

    return 0;
}
