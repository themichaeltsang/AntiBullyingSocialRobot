struct Vector3
{
  Vector3(float x, float y, float z) : x_(x), y_(y), z_(z) { }
  float x_, y_, z_;
};
struct Vector4
{
  Vector4(float w, float x, float y, float z) : w_(w), x_(x), y_(y), z_(z) {} 
  float w_, x_, y_, z_;
};

struct KinectHDFaceData
{
  enum class AnimationUnit
  {
    JAW_OPEN = 0,
    LIP_PUCKER = 1,
    JAW_SLIDE_RIGHT = 2, 
    LIP_STRETCHER_RIGHT = 3,
    LIP_STRETCHER_LEFT = 4,
    LIP_CORNERPULLER_LEFT = 5,
    LIP_CORNERPULLER_RIGHT = 6,
    LIP_CORNERDEPRESSOR_LEFT = 7,
    LIP_CORNERDEPRESSOR_RIGHT = 8,
    LEFT_CHEEK_PUFF = 9,
    RIGHT_CHEEK_PUFF = 10,
    LEFT_EYE_CLOSED = 11,
    RIGHT_EYE_CLOSED = 12,
    RIGHT_EYEBROW_LOWERER = 13,
    LEFT_EYEBROW_LOWERER = 14,
    LOWERLIP_DEPRESSOR_LEFT = 15,
    LOWERLIP_DEPRESSOR_RIGHT = 16
  };

  static std::map<AnimationUnit, std::string> buildAnimationUnitMap()
  {
      std::map<AnimationUnit, std::string> animation_unit_map;
      animation_unit_map[AnimationUnit::JAW_OPEN] = "au_jaw_open";
      animation_unit_map[AnimationUnit::LIP_PUCKER] = "au_lip_pucker";
      animation_unit_map[AnimationUnit::JAW_SLIDE_RIGHT] = "au_jaw_slide_right";
      animation_unit_map[AnimationUnit::LIP_STRETCHER_RIGHT] = "au_lip_stretcher_right";
      animation_unit_map[AnimationUnit::LIP_STRETCHER_LEFT] = "au_lip_stretcher_left";
      animation_unit_map[AnimationUnit::LIP_CORNERPULLER_LEFT] = "au_lip_cornerpuller_left";
      animation_unit_map[AnimationUnit::LIP_CORNERPULLER_RIGHT] = "au_lip_cornerpuller_right";
      animation_unit_map[AnimationUnit::LIP_CORNERDEPRESSOR_LEFT] = "au_lip_cornerdepressor_left";
      animation_unit_map[AnimationUnit::LIP_CORNERDEPRESSOR_RIGHT] = "au_lip_cornerdepressor_right";
      animation_unit_map[AnimationUnit::LEFT_CHEEK_PUFF] = "au_left_cheek_puff";
      animation_unit_map[AnimationUnit::RIGHT_CHEEK_PUFF] = "au_right_cheek_puff";
      animation_unit_map[AnimationUnit::LEFT_EYE_CLOSED] = "au_left_eye_closed";
      animation_unit_map[AnimationUnit::RIGHT_EYE_CLOSED] = "au_right_eye_closed";
      animation_unit_map[AnimationUnit::RIGHT_EYEBROW_LOWERER] = "au_right_eyebrow_lowerer";
      animation_unit_map[AnimationUnit::LEFT_EYEBROW_LOWERER] = "au_left_eyebrow_lowerer";      
      animation_unit_map[AnimationUnit::LOWERLIP_DEPRESSOR_LEFT] = "au_lowerlip_depressor_left";
      animation_unit_map[AnimationUnit::LOWERLIP_DEPRESSOR_RIGHT] = "au_lowerlip_depressor_right";
      return animation_unit_map;
  }
};


struct KinectFaceData
{
  enum class FaceProperty
  {
    HAPPY = 0,
    ENGAGED = 1,
    WEARING_GLASSES = 2, 
    LEFT_EYE_CLOSED = 3,
    RIGHT_EYE_CLOSED = 4,
    MOUTH_OPEN = 5,
    MOUTH_MOVED = 6,
    LOOKING_AWAY = 7
  };


  static std::map<FaceProperty, std::string> buildFacePropertiesMap()
  {
      std::map<FaceProperty, std::string> property_names_map;
      property_names_map[FaceProperty::HAPPY] = "happy";
      property_names_map[FaceProperty::ENGAGED] = "engaged";
      property_names_map[FaceProperty::WEARING_GLASSES] = "wearing_glasses";
      property_names_map[FaceProperty::LEFT_EYE_CLOSED] = "left_eye_closed";
      property_names_map[FaceProperty::RIGHT_EYE_CLOSED] = "right_eye_closed";
      property_names_map[FaceProperty::MOUTH_OPEN] = "mouth_open";
      property_names_map[FaceProperty::MOUTH_MOVED] = "mouth_moved";
      property_names_map[FaceProperty::LOOKING_AWAY] = "looking_away";

      return property_names_map;
  }

};
struct KinectBodyLeanData
{
  KinectBodyLeanData(float x, float y ) : x_(x), y_(y) { }
  float x_, y_;
};
struct KinectJointData
{
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

  static std::map<JointType, std::string> buildJointNamesMap()
  {
      std::map<JointType, std::string> joint_names_map;
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

  KinectJointData(Vector3 position, Vector4 orientation, JointType joint_type, TrackingState tracking_state) : position_(position), orientation_(orientation), joint_type_(joint_type), tracking_state_(tracking_state)  { }

  Vector3 position_;
  Vector4 orientation_;

  JointType joint_type_;
  TrackingState tracking_state_;
};

struct KinectBodyData
{
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

  KinectBodyData(uint32_t std_body_idx, 
                 uint32_t frame, 
                 //uint64_t tracking_id, 
                 float time_stamp, 
                 std::vector<KinectJointData>& joints, 
                 KinectBodyLeanData lean, 
                 //uint8_t is_tracked, 
                 HandState hand_state_left, 
                 HandState hand_state_right, 
                 TrackingConfidence hand_left_confidence,
                 TrackingConfidence hand_right_confidence,
                 uint32_t clipped_edges,
                 uint8_t is_restricted) 
  : 
  std_body_idx_(std_body_idx), 
  frame_(frame), 
  time_stamp_(time_stamp), 
  joints_(joints), 
  lean_(lean), 
  //is_tracked_(is_tracked),
  hand_state_left_(hand_state_left), 
  hand_state_right_(hand_state_right),
  hand_left_confidence_(hand_left_confidence),
  hand_right_confidence_(hand_right_confidence),
  clipped_edges_(clipped_edges),
  is_restricted_(is_restricted) { }

  uint32_t std_body_idx_;
  uint32_t frame_;
  //uint64_t tracking_id_;
  float time_stamp_;

  std::vector<KinectJointData> & joints_;
  KinectBodyLeanData lean_;
  //uint8_t is_tracked_;
  HandState hand_state_left_;
  HandState hand_state_right_;
  TrackingConfidence hand_left_confidence_;
  TrackingConfidence hand_right_confidence_;
  uint32_t clipped_edges_;
  uint8_t is_restricted_;
};

struct CLMFacialActionUnitData
{
  enum class AUType
  {
      AU01_R = 0,
      AU04_R = 1,
      AU06_R = 2,
      AU10_R = 3,
      AU12_R = 4,
      AU14_R = 5,
      AU17_R = 6,
      AU25_R = 7,
      AU02_R = 8,
      AU05_R = 9,
      AU09_R = 10,
      AU15_R = 11,
      AU20_R = 12,
      AU26_R = 13,
      AU12_C = 14,
      AU23_C = 15,
      AU28_C = 16,
      AU04_C = 17,
      AU15_C = 18,
      AU45_C = 19
  };

  static std::map<AUType, std::pair<std::string,std::string> > buildActionUnitIDsMap()
  {
      std::map<AUType, std::pair<std::string,std::string>> au_ids_map;
      au_ids_map[AUType::AU01_R] = std::make_pair("AU01","regression");
      au_ids_map[AUType::AU04_R] = std::make_pair("AU04","regression");
      au_ids_map[AUType::AU06_R] = std::make_pair("AU06","regression");
      au_ids_map[AUType::AU10_R] = std::make_pair("AU10","regression");
      au_ids_map[AUType::AU12_R] = std::make_pair("AU12","regression");
      au_ids_map[AUType::AU14_R] = std::make_pair("AU14","regression");
      au_ids_map[AUType::AU17_R] = std::make_pair("AU17","regression");
      au_ids_map[AUType::AU25_R] = std::make_pair("AU25","regression");
      au_ids_map[AUType::AU02_R] = std::make_pair("AU02","regression");
      au_ids_map[AUType::AU05_R] = std::make_pair("AU05","regression");
      au_ids_map[AUType::AU09_R] = std::make_pair("AU09","regression");
      au_ids_map[AUType::AU15_R] = std::make_pair("AU15","regression");
      au_ids_map[AUType::AU20_R] = std::make_pair("AU20","regression");
      au_ids_map[AUType::AU26_R] = std::make_pair("AU26","regression");
      au_ids_map[AUType::AU12_C] = std::make_pair("AU12","classification");
      au_ids_map[AUType::AU23_C] = std::make_pair("AU23","classification");
      au_ids_map[AUType::AU28_C] = std::make_pair("AU28","classification");
      au_ids_map[AUType::AU04_C] = std::make_pair("AU04","classification");
      au_ids_map[AUType::AU15_C] = std::make_pair("AU15","classification");
      au_ids_map[AUType::AU45_C] = std::make_pair("AU45","classification");
      return au_ids_map;
  }
  CLMFacialActionUnitData(std::string au_id, 
                          float au_value, 
                          std::string au_prediction_method) 
  : au_id_(au_id), 
    au_value_(au_value), 
    au_prediction_method_(au_prediction_method) { }

  std::string au_id_; //the ID of the action unit
  float au_value_; //the regression or classification value of the action unit
  std::string au_prediction_method_;
};

struct CLMHeadData
{

  
  CLMHeadData(uint32_t head_idx,
              uint8_t conflicted_body_idx,
              float detection_confidence,
              bool detection_success,
              uint32_t frame,
              float time_stamp,
              Vector3 headpose_translation, 
              Vector3 headpose_rotation,
              Vector3 right_eyegaze_cameraref,
              Vector3 left_eyegaze_cameraref,
              Vector3 right_eyegaze_headref,
              Vector3 left_eyegaze_headref,
              std::vector<CLMFacialActionUnitData> & facs) 
  :  head_idx_(head_idx),
     conflicted_body_idx_(conflicted_body_idx),
     detection_confidence_(detection_confidence),
     detection_success_(detection_success),
     frame_(frame),
     time_stamp_(time_stamp),
     headpose_translation_(headpose_translation),
     headpose_rotation_(headpose_rotation),
     right_eyegaze_cameraref_(right_eyegaze_cameraref),
     left_eyegaze_cameraref_(left_eyegaze_cameraref),
     right_eyegaze_headref_(right_eyegaze_headref),
     left_eyegaze_headref_(left_eyegaze_headref),
     facs_(facs)
  {}

  uint32_t head_idx_;
  uint8_t conflicted_body_idx_;
  float detection_confidence_;
  bool detection_success_;
  uint32_t frame_;
  float time_stamp_;

  //headpose
  Vector3 headpose_translation_ ; 
  Vector3 headpose_rotation_;

  //eyegaze
  Vector3 right_eyegaze_cameraref_;
  Vector3 left_eyegaze_cameraref_;

  Vector3 right_eyegaze_headref_;
  Vector3 left_eyegaze_headref_;

  //facs
  std::vector<CLMFacialActionUnitData> & facs_;
};