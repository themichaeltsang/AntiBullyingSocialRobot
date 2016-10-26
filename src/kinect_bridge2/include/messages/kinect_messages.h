#ifndef _MESSAGES_KINECTMESSAGES_H_
#define _MESSAGES_KINECTMESSAGES_H_

#include <map>
#include <limits>

#include <messages/container_messages.h>
#include <messages/utility_messages.h>
#include <messages/image_message.h>
#include <messages/audio_message.h>
#include <messages/geometry_messages.h>

// ####################################################################################################
template<class __ImageMessage = ImageMessage<> >
class KinectColorImageMessage : public MultiMessage<__ImageMessage, TimeStampMessage>
{
public:
    typedef MultiMessage<__ImageMessage, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectColorImageMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectColorImageMessage )
};

// ####################################################################################################
class KinectDepthImageInfoMessage : public SerializableInterface
{
public:
    uint16_t min_reliable_distance_;
    uint16_t max_reliable_distance_;

    // ====================================================================================================
    KinectDepthImageInfoMessage()
    :
        min_reliable_distance_( 0 ),
        max_reliable_distance_( 0 )
    {
        //
    }

    // ====================================================================================================
    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        archive << min_reliable_distance_;
        archive << max_reliable_distance_;
    }

    // ====================================================================================================
    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> min_reliable_distance_;
        archive >> max_reliable_distance_;
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectDepthImageInfoMessage )
};

// ####################################################################################################
template<class __ImageMessage = ImageMessage<> >
class KinectDepthImageMessage : public MultiMessage<__ImageMessage, KinectDepthImageInfoMessage, TimeStampMessage>
{
public:
    typedef MultiMessage<__ImageMessage, KinectDepthImageInfoMessage, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectDepthImageMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectDepthImageMessage )
};

// ####################################################################################################
template<class __ImageMessage = ImageMessage<> >
class KinectInfraredImageMessage : public MultiMessage<__ImageMessage, TimeStampMessage>
{
public:
    typedef MultiMessage<__ImageMessage, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectInfraredImageMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectInfraredImageMessage )
};

// ####################################################################################################
class KinectAudioInfoMessage : public SerializableInterface
{
public:
    float beam_angle_;
    float beam_angle_confidence_;

    // ====================================================================================================
    KinectAudioInfoMessage( float beam_angle = 0, float beam_angle_confidence = 0 )
    :
        beam_angle_( beam_angle ),
        beam_angle_confidence_( beam_angle_confidence )
    {
        //
    }

    // ====================================================================================================
    template<class __Archive>
    void pack( __Archive & archive ) const
    {
        archive << beam_angle_;
        archive << beam_angle_confidence_;
    }

    // ====================================================================================================
    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> beam_angle_;
        archive >> beam_angle_confidence_;
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectAudioInfoMessage )
};

// ####################################################################################################
template<class __AudioMessage = AudioMessage<> >
class KinectAudioMessage : public MultiMessage<__AudioMessage, KinectAudioInfoMessage, TimeStampMessage>
{
public:
    typedef MultiMessage<__AudioMessage, KinectAudioInfoMessage, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectAudioMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectAudioMessage )
};


// ####################################################################################################
class KinectRawAudioInfoMessage : public SerializableInterface
{
public:
	uint8_t is_tracked_;
	float energy_;
	float time_stamp_;
	uint32_t frame_;

	// ====================================================================================================
	KinectRawAudioInfoMessage(uint8_t is_tracked = 0 , float energy = -std::numeric_limits<float>::infinity(), float time_stamp = 0)
	:
		is_tracked_(is_tracked),
		energy_(energy),
		time_stamp_(time_stamp),
		frame_(0)
	{
		//
	}

	// ====================================================================================================
	template<class __Archive>
	void pack(__Archive & archive) const
	{
		archive << is_tracked_;
		archive << energy_;
		archive << frame_;
		archive << time_stamp_;
	}

	// ====================================================================================================
	template<class __Archive>
	void unpack(__Archive & archive)
	{
		archive >> is_tracked_;
		archive >> energy_;
		archive >> frame_;
		archive >> time_stamp_;
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectRawAudioInfoMessage)
};

// ####################################################################################################
class KinectRawAudioMessage : public MultiMessage<KinectRawAudioInfoMessage, TimeStampMessage>
{
public:
	typedef MultiMessage<KinectRawAudioInfoMessage, TimeStampMessage> _Message;

	// ====================================================================================================
	template<class... __Args>
	KinectRawAudioMessage(__Args&&... args)
		:
		_Message(std::forward<__Args>(args)...)
	{
		//
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectRawAudioMessage)
};

// ####################################################################################################
class KinectJointMessage : public SerializableInterface
{
public:
    enum class JointType
    {
        SPINE_BASE = 0,
        SPINE_MID = 1,
        NECK = 2,
        HEAD = 3,
        SHOULDER_LEFT = 4,
        ELBOW_LEFT = 5,
        WRIST_LEFT = 6,
        HAND_LEFT = 7,
        SHOULDER_RIGHT = 8,
        ELBOW_RIGHT = 9,
        WRIST_RIGHT = 10,
        HAND_RIGHT = 11,
        HIP_LEFT = 12,
        KNEE_LEFT = 13,
        ANKLE_LEFT = 14,
        FOOT_LEFT = 15,
        HIP_RIGHT = 16,
        KNEE_RIGHT = 17,
        ANKLE_RIGHT = 18,
        FOOT_RIGHT = 19,
        SPINE_SHOULDER = 20,
        HANDTIP_LEFT = 21,
        THUMB_LEFT = 22,
        HANDTIP_RIGHT = 23,
        THUMB_RIGHT = 24
    };

    enum class TrackingState
    {
        NOT_TRACKED = 0,
        INFERRED = 1,
        TRACKED = 2
    };

    typedef std::map<JointType, std::string> _JointNamesMap;

    static _JointNamesMap buildJointNamesMap()
    {
        _JointNamesMap joint_names_map;
        joint_names_map[JointType::SPINE_BASE] = "spine_base";
        joint_names_map[JointType::SPINE_MID] = "spine_mid";
        joint_names_map[JointType::NECK] = "neck";
        joint_names_map[JointType::HEAD] = "head";
        joint_names_map[JointType::SHOULDER_LEFT] = "shoulder_left";
        joint_names_map[JointType::ELBOW_LEFT] = "elbow_left";
        joint_names_map[JointType::WRIST_LEFT] = "wrist_left";
        joint_names_map[JointType::HAND_LEFT] = "hand_left";
        joint_names_map[JointType::SHOULDER_RIGHT] = "shoulder_right";
        joint_names_map[JointType::ELBOW_RIGHT] = "elbow_right";
        joint_names_map[JointType::WRIST_RIGHT] = "wrist_right";
        joint_names_map[JointType::HAND_RIGHT] = "hand_right";
        joint_names_map[JointType::HIP_LEFT] = "hip_left";
        joint_names_map[JointType::KNEE_LEFT] = "knee_left";
        joint_names_map[JointType::ANKLE_LEFT] = "ankle_left";
        joint_names_map[JointType::FOOT_LEFT] = "foot_left";
        joint_names_map[JointType::HIP_RIGHT] = "hip_right";
        joint_names_map[JointType::KNEE_RIGHT] = "knee_right";
        joint_names_map[JointType::ANKLE_RIGHT] = "ankle_right";
        joint_names_map[JointType::FOOT_RIGHT] = "foot_right";
        joint_names_map[JointType::SPINE_SHOULDER] = "spine_shoulder";
        joint_names_map[JointType::HANDTIP_LEFT] = "handtip_left";
        joint_names_map[JointType::THUMB_LEFT] = "thumb_left";
        joint_names_map[JointType::HANDTIP_RIGHT] = "handtip_right";
        joint_names_map[JointType::THUMB_RIGHT] = "thumb_right";
        return joint_names_map;
    }

    static _JointNamesMap const & getJointNamesMap()
    {
        static _JointNamesMap const & joint_names_map( buildJointNamesMap() );

        return joint_names_map;
    }

    PointMessage<float, 3> position_;
    PointMessage<float, 4> orientation_;

    JointType joint_type_;
    TrackingState tracking_state_;

    // ====================================================================================================
    KinectJointMessage()
    {
        //
    }

    // ====================================================================================================
    template<class __Archive>
    void pack( __Archive & archive )
    {
        archive << static_cast<uint8_t>( joint_type_ );
        archive << static_cast<uint8_t>( tracking_state_ );
        position_.pack( archive );
        orientation_.pack( archive );
    }

    // ====================================================================================================
    template<class __Archive>
    void unpack( __Archive & archive )
    {
        uint8_t joint_type;
        archive >> joint_type;
        joint_type_ = static_cast<JointType>( joint_type );

        uint8_t tracking_state;
        archive >> tracking_state;
        tracking_state_ = static_cast<TrackingState>( tracking_state );

        position_.unpack( archive );
        orientation_.unpack( archive );
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectJointMessage )
};

// ####################################################################################################
class KinectBodyLeanMessage : public SerializableInterface
{
public:
	float x, y;

	KinectBodyLeanMessage()
	{
		x = 0.;
		y = 0.;

	}

	// ====================================================================================================
	template<class __Archive>
	void pack(__Archive & archive)
	{
		archive << static_cast<float>(x);
		archive << static_cast<float>(y);
	}

	// ====================================================================================================
	template<class __Archive>
	void unpack(__Archive & archive)
	{
		float _x;
		archive >> _x;
		x = static_cast<float>(_x);

		float _y;
		archive >> _y;
		y = static_cast<float>(_y);
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectBodyLeanMessage)
};

// ####################################################################################################
class KinectBodyMessage : public VectorMessage<KinectJointMessage>
{
public:
    typedef VectorMessage<KinectJointMessage> _Message;

    enum class HandState
    {
        UNKNOWN = 0,
        NOT_TRACKED = 1,
        OPEN = 2,
        CLOSED = 3,
        LASSO = 4
    };

	enum class TrackingConfidence
	{
		TrackingConfidence_Low = 0,
		TrackingConfidence_High = 1
	};

	uint32_t std_body_idx_;
	uint32_t frame_;
	uint64_t tracking_id_;
	float time_stamp_;

    std::vector<KinectJointMessage> & joints_;
	KinectBodyLeanMessage lean_;
    uint8_t is_tracked_;
    HandState hand_state_left_;
    HandState hand_state_right_;
	TrackingConfidence hand_left_confidence_;
	TrackingConfidence hand_right_confidence_;
	uint32_t clipped_edges_;
	uint8_t is_restricted_;



    // ====================================================================================================
    KinectBodyMessage()
    :
        _Message(),
		std_body_idx_( 0 ),
        joints_( this->payload_ ),
		lean_(),
        is_tracked_( 0 ),
        hand_state_left_( HandState::UNKNOWN ),
        hand_state_right_( HandState::UNKNOWN ),
		hand_left_confidence_( TrackingConfidence::TrackingConfidence_Low ),
		hand_right_confidence_( TrackingConfidence::TrackingConfidence_Low ),
		clipped_edges_( 0 ),
		is_restricted_( 0 ),
        tracking_id_( 0 ),
		time_stamp_( 0 ),
		frame_( 0 )
    {
        //
    }

    // ====================================================================================================
    template<class __Archive>
    void pack( __Archive & archive )
    {
		archive << std_body_idx_;
        archive << is_tracked_;
		archive << frame_;
		archive << static_cast<float>(time_stamp_);
        archive << static_cast<uint8_t>( hand_state_left_ );
        archive << static_cast<uint8_t>( hand_state_right_ );
		archive << static_cast<uint8_t>( hand_left_confidence_ );
		archive << static_cast<uint8_t>( hand_right_confidence_ );
		archive << clipped_edges_ ;
		archive << is_restricted_;

        archive << tracking_id_;
		lean_.pack(archive);
        _Message::pack( archive );
    }

    // ====================================================================================================
    template<class __Archive>
    void unpack( __Archive & archive )
    {
		archive >> std_body_idx_;
        archive >> is_tracked_;
		archive >> frame_;

		archive >> time_stamp_; 

        uint8_t hand_state_left;
        archive >> hand_state_left;
        hand_state_left_ = static_cast<HandState>( hand_state_left );

        uint8_t hand_state_right;
        archive >> hand_state_right;
        hand_state_right_ = static_cast<HandState>( hand_state_right );

		uint8_t hand_left_confidence;
		archive >> hand_left_confidence;
		hand_left_confidence_ = static_cast<TrackingConfidence>( hand_left_confidence );

		uint8_t hand_right_confidence;
		archive >> hand_right_confidence;
		hand_right_confidence_ = static_cast<TrackingConfidence>( hand_right_confidence );
		archive >> clipped_edges_;
		archive >> is_restricted_;

        archive >> tracking_id_;
		lean_.unpack(archive);
        _Message::unpack( archive );
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectBodyMessage )
};

// ####################################################################################################
class KinectBodiesMessage : public MultiMessage<VectorMessage<KinectBodyMessage>, TimeStampMessage>
{
public:
    typedef MultiMessage<VectorMessage<KinectBodyMessage>, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectBodiesMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectBodiesMessage )
};

// ####################################################################################################
class KinectSpeechPhraseMessage : public SerializableInterface
{
public:
    std::string tag_;
    float confidence_;

    // ====================================================================================================
    KinectSpeechPhraseMessage()
    {
        //
    }

    // ====================================================================================================
    template<class __Archive>
    void pack( __Archive & archive )
    {
        archive << tag_;
        archive << static_cast<float>( confidence_ );
    }

    // ====================================================================================================
    template<class __Archive>
    void unpack( __Archive & archive )
    {
        archive >> tag_;
        archive >> confidence_;
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectSpeechPhraseMessage )
};

// ####################################################################################################
class KinectSpeechMessage : public MultiMessage<VectorMessage<KinectSpeechPhraseMessage>, TimeStampMessage>
{
public:
    typedef MultiMessage<VectorMessage<KinectSpeechPhraseMessage>, TimeStampMessage> _Message;

    // ====================================================================================================
    template<class... __Args>
    KinectSpeechMessage( __Args&&... args )
    :
        _Message( std::forward<__Args>( args )... )
    {
        //
    }

    // ====================================================================================================
    DECLARE_MESSAGE_INFO( KinectSpeechMessage )
};


// ####################################################################################################

class KinectFacePropertyMessage : public SerializableInterface
{
public:
	enum class DetectionResult
	{
		DetectionResult_Unknown = 0,
		DetectionResult_No = 1,
		DetectionResult_Maybe = 2,
		DetectionResult_Yes = 3
	};

	std::string property_name_; 
	DetectionResult property_value_; 

	// ====================================================================================================
	KinectFacePropertyMessage()
	{
		//
	}

	// ====================================================================================================
	template<class __Archive>
	void pack(__Archive & archive)
	{
		archive << property_name_;
		archive << static_cast<uint8_t>(property_value_);
	}

	// ====================================================================================================
	template<class __Archive>
	void unpack(__Archive & archive)
	{
		archive >> property_name_;

		uint8_t property_value;
		archive >> property_value;
		property_value_ = static_cast<DetectionResult>(property_value);
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectFacePropertyMessage)
};

class KinectFaceMessage : public VectorMessage<KinectFacePropertyMessage>
{
public:
	typedef VectorMessage<KinectFacePropertyMessage> _Message;

	//float detection_confidence_; //
	uint32_t std_body_idx_;
	uint8_t is_tracked_; // face_istracked
	float time_stamp_;
	uint32_t frame_;

	//headpose
	PointMessage<int, 3> headpose_rotation_;

	//face properties
	std::vector<KinectFacePropertyMessage> & properties_;

	// ====================================================================================================
	KinectFaceMessage() :
		_Message(),
		properties_(this->payload_),
		//detection_confidence_(0),
		is_tracked_(0),
		std_body_idx_(0),
		time_stamp_(0),
		frame_(0)
	{
		//
	}

	// ====================================================================================================
	template<class __Archive>
	void pack(__Archive & archive)
	{
		//archive << static_cast<float>(detection_confidence_);
		archive << std_body_idx_;
		archive << is_tracked_;
		archive << frame_;
		archive << static_cast<float>(time_stamp_);

		headpose_rotation_.pack(archive);

		_Message::pack(archive);
	}

	// ====================================================================================================
	template<class __Archive>
	void unpack(__Archive & archive)
	{
		//archive >> detection_confidence_; //??
		archive >> std_body_idx_;
		archive >> is_tracked_;
		archive >> frame_;
		archive >> time_stamp_;

		headpose_rotation_.unpack(archive);
		_Message::unpack(archive);
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectFaceMessage)
};

class KinectFacesMessage : public MultiMessage<VectorMessage<KinectFaceMessage>, TimeStampMessage>
{
public:
	typedef MultiMessage<VectorMessage<KinectFaceMessage>, TimeStampMessage> _Message;

	// ====================================================================================================
	template<class... __Args>
	KinectFacesMessage(__Args&&... args)
		:
		_Message(std::forward<__Args>(args)...)
	{
		//
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectFacesMessage)
};

// ####################################################################################################

class KinectAnimationUnitMessage : public SerializableInterface
{
public:
	std::string name_; //the ID of the animation unit
	float value_; //the regression value of the animation unit

	// ====================================================================================================
	KinectAnimationUnitMessage()
	{
		//
	}

	// ====================================================================================================
	template<class __Archive>
	void pack(__Archive & archive)
	{
		archive << name_;
		archive << value_;
	}

	// ====================================================================================================
	template<class __Archive>
	void unpack(__Archive & archive)
	{
		archive >> name_;
		archive >> value_;
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectAnimationUnitMessage)
};

class KinectHDFaceMessage : public VectorMessage<KinectAnimationUnitMessage>
{
public:
	typedef VectorMessage<KinectAnimationUnitMessage> _Message;

	//float detection_confidence_; //
	uint32_t std_body_idx_;
	uint8_t is_tracked_; // face_istracked
	uint32_t frame_;
	float time_stamp_;

	uint8_t tongue_sticking_out_;
	uint16_t tongue_depth_;
	uint16_t upper_lip_depth_;

	//headpose
	PointMessage<int, 3> headpose_rotation_;
	PointMessage<int, 4> bounding_box_;
	PointMessage<float, 3> head_pivot_;

	uint8_t face_quality_;

	//face properties
	std::vector<KinectAnimationUnitMessage> & animation_units_;

	// ====================================================================================================
	KinectHDFaceMessage() :
		_Message(),
		animation_units_(this->payload_),
		//detection_confidence_(0),
		is_tracked_(0),
		std_body_idx_(0),
		frame_(0),
		time_stamp_(0),
		face_quality_(1),
		tongue_sticking_out_(0),
		tongue_depth_(0),
		upper_lip_depth_(0)
	{
		//
	}

	// ====================================================================================================
	template<class __Archive>
	void pack(__Archive & archive)
	{
		//archive << static_cast<float>(detection_confidence_);
		archive << std_body_idx_;
		archive << is_tracked_;
		archive << frame_;
		archive << static_cast<float>(time_stamp_);

		archive << tongue_sticking_out_;
		archive << tongue_depth_;
		archive << upper_lip_depth_;

		headpose_rotation_.pack(archive);
		bounding_box_.pack(archive);
		head_pivot_.pack(archive);

		_Message::pack(archive);
	}

	// ====================================================================================================
	template<class __Archive>
	void unpack(__Archive & archive)
	{
		//archive >> detection_confidence_; //??
		archive >> std_body_idx_;
		archive >> is_tracked_;
		archive >> frame_;
		archive >> time_stamp_;

		archive >> tongue_sticking_out_;
		archive >> tongue_depth_;
		archive >> upper_lip_depth_;

		headpose_rotation_.unpack(archive);
		bounding_box_.unpack(archive);
		head_pivot_.unpack(archive);

		_Message::unpack(archive);
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectHDFaceMessage)
};

class KinectHDFacesMessage : public MultiMessage<VectorMessage<KinectHDFaceMessage>, TimeStampMessage>
{
public:
	typedef MultiMessage<VectorMessage<KinectHDFaceMessage>, TimeStampMessage> _Message;

	// ====================================================================================================
	template<class... __Args>
	KinectHDFacesMessage(__Args&&... args)
		:
		_Message(std::forward<__Args>(args)...)
	{
		//
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(KinectHDFacesMessage)
};


// ####################################################################################################
class CLMFacialActionUnitMessage : public SerializableInterface
{
public:
	//enum class ActionUnitType
	//{
	//	R_AU01_INNER_BROW_RAISER = 0,
	//	R_AU02_OUTER_BROW_RAISER = 1,
	//	R_AU04_BROW_LOWERER = 2,
	//	R_AU05_UPPER_LID_RAISER = 3,
	//	R_AU06_CHEEK_RAISER = 4,
	//	R_AU09_NOSE_WRINKLER = 5,
	//	R_AU10_UPPER_LIP_RAISER = 6,
	//	R_AU12_LIP_CORNER_PULLER = 7,
	//	R_AU14_DIMPLER = 8,
	//	R_AU15_LIP_CORNER_DEPRESSOR = 9,
	//	R_AU17_CHIN_RAISER = 10,
	//	R_AU20_LIP_STRETCHER = 11,
	//	R_AU25_LIPS_PART = 12,
	//	R_AU26_JAW_DROP = 13,

	//	C_AU04_BROW_LOWERER = 14,
	//	C_AU12_LIP_CORNER_PULLER = 15,
	//	C_AU15_LIP_CORNER_DEPRESSOR = 16,
	//	C_AU23_LIP_TIGHTENER = 17,
	//	C_AU28_LIP_SUCK = 18,
	//	C_AU45_BLINK = 19
	//};

	std::string au_id_; //the ID of the action unit
	float au_value_; //the regression or classification value of the action unit
	std::string au_prediction_method_;
	//int au_id_; //the ID of the action unit

	// ====================================================================================================
	CLMFacialActionUnitMessage()
	{
		//
	}

	// ====================================================================================================
	template<class __Archive>
	void pack(__Archive & archive)
	{
		//archive << static_cast<float>(au_id_);
		
		/*archive << static_cast<std::string>(au_id_);
		archive << static_cast<float>(au_value_);
		archive << static_cast<std::string>(au_prediction_method_);
		*/
		archive << au_id_;
		archive << au_value_;
		archive << au_prediction_method_;
	}

	// ====================================================================================================
	template<class __Archive>
	void unpack(__Archive & archive)
	{
		//float au_id;
		//archive >> au_id;
		//au_id_ = static_cast<ActionUnitType>(au_id);

		archive >> au_id_;
		archive >> au_value_;
		archive >> au_prediction_method_;
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(CLMFacialActionUnitMessage)
};

class CLMHeadMessage : public VectorMessage<CLMFacialActionUnitMessage>
{
public:
	typedef VectorMessage<CLMFacialActionUnitMessage> _Message;
	
	uint32_t std_body_idx_;
	uint8_t conflicted_body_idx_;
	float detection_confidence_;
	bool detection_success_;
	uint32_t frame_;
	float time_stamp_;

	//headpose
	PointMessage<float, 3> headpose_translation_ ;
	PointMessage<float, 3> headpose_rotation_;

	//eyegaze
	PointMessage<float, 3> right_eyegaze_cameraref_;
	PointMessage<float, 3> left_eyegaze_cameraref_;

	PointMessage<float, 3> right_eyegaze_headref_;
	PointMessage<float, 3> left_eyegaze_headref_;

	//facs
	std::vector<CLMFacialActionUnitMessage> & facs_;

	// ====================================================================================================
	CLMHeadMessage() :
		_Message(),
		std_body_idx_(0),
		conflicted_body_idx_(0),
		facs_(this->payload_),
		detection_confidence_(0),
		detection_success_(0),
		frame_(0),
		time_stamp_(0)
	{
		//
	}

	// ====================================================================================================
	template<class __Archive>
	void pack(__Archive & archive)
	{
		archive << std_body_idx_;
		archive << conflicted_body_idx_;
		archive << static_cast<float>(detection_confidence_);
		archive << static_cast<bool>(detection_success_);
		archive << frame_;
		archive << static_cast<float>(time_stamp_);

		headpose_translation_.pack(archive);
		headpose_rotation_.pack(archive);
		right_eyegaze_cameraref_.pack(archive);
		left_eyegaze_cameraref_.pack(archive);
		right_eyegaze_headref_.pack(archive);
		left_eyegaze_headref_.pack(archive);

		_Message::pack(archive);
	}

	// ====================================================================================================
	template<class __Archive>
	void unpack(__Archive & archive)
	{
		archive >> std_body_idx_;
		archive >> conflicted_body_idx_;
		archive >> detection_confidence_;
		archive >> detection_success_;
		archive >> frame_;
		archive >> time_stamp_;

		headpose_translation_.unpack(archive);
		headpose_rotation_.unpack(archive);
		right_eyegaze_cameraref_.unpack(archive);
		left_eyegaze_cameraref_.unpack(archive);
		right_eyegaze_headref_.unpack(archive);
		left_eyegaze_headref_.unpack(archive);

		_Message::unpack(archive);
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(CLMHeadMessage)
};

class CLMHeadsMessage : public MultiMessage<VectorMessage<CLMHeadMessage>, TimeStampMessage>
{
public:
	typedef MultiMessage<VectorMessage<CLMHeadMessage>, TimeStampMessage> _Message;

	// ====================================================================================================
	template<class... __Args>
	CLMHeadsMessage(__Args&&... args)
		:
		_Message(std::forward<__Args>(args)...)
	{
		//
	}

	// ====================================================================================================
	DECLARE_MESSAGE_INFO(CLMHeadsMessage)
};


#endif // _MESSAGES_KINECTMESSAGES_H_