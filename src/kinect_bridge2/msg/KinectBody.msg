uint8 HAND_STATE_UNKNOWN = 0
uint8 HAND_STATE_NOT_TRACKED = 1
uint8 HAND_STATE_OPEN = 2
uint8 HAND_STATE_CLOSED = 3
uint8 HAND_STATE_LASSO = 4

kinect_bridge2/KinectJoint[] joints
uint8 is_tracked
uint32 frame
uint32 time_stamp
uint32 std_body_idx
uint64 tracking_id

uint8 hand_state_left
uint8 hand_state_right
uint8 hand_left_confidence
uint8 hand_right_confidence
uint32 clipped_edges
uint8 is_restricted

float32 proxemics

geometry_msgs/Point head_position
geometry_msgs/Point spine_mid_position
kinect_bridge2/KinectBodyLean lean
kinect_bridge2/KinectBodyOrientation orientation
kinect_bridge2/KinectBodyPose pose
