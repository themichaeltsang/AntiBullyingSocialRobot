/*******************************************************************************
 *
 *      bandit_node
 *
 *      Copyright (c) 2011, edward
 *      All rights reserved.
 *
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions are
 *      met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following disclaimer
 *        in the documentation and/or other materials provided with the
 *        distribution.
 *      * Neither the name of "interaction-ros-pkg" nor the names of its
 *        contributors may be used to endorse or promote products derived from
 *        this software without specific prior written permission.
 *
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *      "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *      LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *      A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *      OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *      SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *      LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *      DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *      THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *      (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *      OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef BANDIT_NODE_H_
#define BANDIT_NODE_H_

#include <ros/ros.h>
#include <bandit_driver/JointArray.h>
#include <bandit_driver/Params.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cstring>
#include <sstream>
#include <chrono>
#include <thread>
#include <libbandit/bandit.h>
#include <libbandit/param_reader.h>
#include <yaml-cpp/yaml.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <string>
#include <vector>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <math.h>


static double degToRad(double in)
{
    return (in * M_PI / 180.);
}

static double radToDeg(double in)
{
    return (in * 180. / M_PI);
}

static double normalizeEuler(double const & value)
{
    double const angle = fmod( value, 2 * M_PI );
    if( angle > M_PI ) return angle - 2 * M_PI;
    else if( angle < -M_PI ) return angle + 2 * M_PI;
    return angle;
}

class BanditNode
{
public:
    bandit::Bandit bandit_driver_;
    bool bandit_state_initialized_;
    bandit_driver::Params::Response param_response_;
    diagnostic_updater::Updater diagnostic_;

    ros::NodeHandle nh_rel;

    ros::Subscriber joint_sub_;
    ros::Subscriber target_sub_;
    ros::ServiceServer service_;

    diagnostic_updater::DiagnosedPublisher<sensor_msgs::JointState> joints_pub_;

    BanditNode( ros::NodeHandle & nh, double desired_freq = 5.0 ) :
                nh_rel( "~" ),
                joints_pub_( nh_rel.advertise<sensor_msgs::JointState> ( "joint_states", 1000 ), diagnostic_, diagnostic_updater::FrequencyStatusParam( &desired_freq, &desired_freq, 0.5 ),
                        diagnostic_updater::TimeStampStatusParam() )
    {
        ROS_INFO( "creating services and subscribing to topics" );

        // Now that things are supposeldy up and running, subscribe to
        // joint messages
        joint_sub_ = nh_rel.subscribe( "bandit_joint_array_cmd", 100, &BanditNode::jointCB, this );
        target_sub_ = nh_rel.subscribe( "joint_state_cmd", 100, &BanditNode::targetCB, this );
        service_ = nh.advertiseService( "params", &BanditNode::paramCB, this );

        // do setup
                // set up bandit driver object

        //auto const port = ros::ParamReader<std::string, 1>::readParam( nh_rel, "port", "/dev/ttyUSB0" );

        std::string port = "/dev/ttyUSB0";//ros::ParamReader<std::string, 1>::readParam( nh_rel, "port", "/dev/ttyUSB0" );


        while( ros::ok() )
        {
            try
            {
                bandit_driver_.openPort( port.c_str() );
                bandit_driver_.useJointLimits( false );
                //bandit_driver_.useJointLimits( ros::ParamReader<bool, 1>::readParam( nh_rel, "use_joint_limits", false ) );
                break;
            }
            catch( std::runtime_error & e )
            {
                ROS_WARN( "Failed to open port: %s", e.what() );
            }

            std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
        }

        // load and parse joint config
        auto & joints_map = bandit_driver_.getJoints();

        int c = 0;
     //    while( ros::ok() ) //make sure the params properly initialize
     //    {
	    //     //auto joints_pid_values = ros::ParamReader<XmlRpc::XmlRpcValue, 1>::readParam( nh_rel, "pids" );
     //        //auto joints_pid_values = ros::ParamReader<XmlRpc::XmlRpcValue, 1>::readParam( nh_rel, "pids" );
	    //     XmlRpc::XmlRpcValue tmp;
	    //     if (ros::ParamReader<XmlRpc::XmlRpcValue, 1>::tryReadParam( nh_rel, "pids", tmp )) //return true if pids initialized
	    //     	break;

	    //     ROS_WARN("Cannot find the pid params... Retrying");
	    //     std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
	    //     c++;
	    //     if (c > 3)
	    //     	break;
	    //     //add 3 second timeout
	    // }
        // read the full set of config values for all joints off of the server at once
        //auto joints_pid_values = ros::ParamReader<XmlRpc::XmlRpcValue, 1>::readParam( nh_rel, "pids" );
        //auto joints_config_values = ros::ParamReader<XmlRpc::XmlRpcValue, 1>::readParam( nh_rel, "joints" );
        for( auto joints_it = joints_map.begin(); joints_it != joints_map.cend(); ++joints_it )
        {
            auto & joint = joints_it->second;

            std::string const & joint_name = joint.name;

            //auto joint_pid_values =    ros::ParamReader<XmlRpc::XmlRpcValue, 1>::getXmlRpcValue( joints_pid_values,    joint_name );
            //auto joint_config_values = ros::ParamReader<XmlRpc::XmlRpcValue, 1>::getXmlRpcValue( joints_config_values, joint_name );

            //ROS_INFO( "%s", joint_config_values.toXml().c_str() );

            if( joint.type == smartservo::SMART_SERVO )
            {
                auto const p =  100;//    ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "p"     , 100   );
                auto const i =  0;//    ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "i"     , 0     );
                auto const d =  0;//    ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "d"     , 0     );
                auto const i_min =  -4000;//ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "i_min" , -4000 );
                auto const i_max =  4001;//ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "i_max" , 4001  );
                auto const e_min =  50;//ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "e_min" , 50    );
                auto const offset = 0;//ros::ParamReader<int, 1>::getXmlRpcValue( joint_pid_values, "offset", 0     );

                joint.setPIDConfig( p, i, d, i_min, i_max, e_min, offset );
            }
            ROS_INFO( "Getting joint direction" );
            auto const direction = 1;//ros::ParamReader<int   , 1>::getXmlRpcValue( joint_config_values, "direction", 1 );
            ROS_INFO( "Getting joint origin" );
            auto const origin = 0;//   ros::ParamReader<double, 1>::getXmlRpcValue( joint_config_values, "origin"   , 0 );

            ROS_INFO( "Setting joint direction and origin" );

            joint.setDirection( direction );
            joint.setOffset( degToRad(origin) ); // DOUBLE CHECK
            // joint.setOffset( Radian( Degree( origin ) ) ); 
        }


        auto const num_joints = bandit_driver_.getNumJoints();
        param_response_.id.reserve( num_joints );
        param_response_.name.reserve( num_joints );
        param_response_.min.reserve( num_joints );
        param_response_.max.reserve( num_joints );
        param_response_.pos.reserve( num_joints );

        for( auto joints_it = joints_map.cbegin(); joints_it != joints_map.cend(); ++joints_it )
        {
            auto const & joint = joints_it->second;
            param_response_.id.push_back( joint.name );
            param_response_.name.push_back( joint.name );
            param_response_.min.push_back( radToDeg( joint.min ) );
            param_response_.max.push_back( radToDeg( joint.max ) );
            param_response_.pos.push_back( radToDeg( joint.getPos() ) );
        }


        ROS_INFO( "Setting and checking PID configs..." );

        ros::Time target_update_time = ros::Time::now();
        do
        {
            auto const now = ros::Time::now();
            if( now >= target_update_time )
            {
                ROS_INFO( "Sending PID info to bandit..." );
                try
                {
                    bandit_driver_.sendAllPIDConfigs();
                }
                catch( std::runtime_error & e )
                {
                    ROS_WARN( "Failed to send PID configs: %s", e.what() );
                }
                target_update_time = now + ros::Duration( 1.0 );
            }

            try
            {
                bandit_driver_.processIO( 10000 );
                //bandit_driver_.processIO( QUICKDEV_GET_RUNABLE_POLICY()::getLoopRateSeconds() * 1000000 );
                bandit_driver_.processPackets();
            }
            catch( std::runtime_error & e )
            {
                ROS_WARN( "Failed to process IO: %s", e.what() );
            }
        }
        while( !bandit_driver_.checkAllPIDConfigs() );


        while( ros::ok() )
        {
            ROS_INFO( "Setting initial joint positions." );

            std::map<std::string, double> defaultPos = 
            {
                //{"head_tilt_joint", deg_to_rad(0)},
                //{"head_pan_joint", deg_to_rad(0)},
                {"left_torso_shoulder_mounting_joint", degToRad(-65)},
                {"left_shoulder_mounting_shoulder_joint", degToRad(-65)},
                {"left_shoulder_bicep_joint", degToRad(-45)},
                {"left_bicep_forearm_joint", degToRad(45)},
                {"left_forearm_wrist_joint", degToRad(0)},
                {"left_wrist_hand_joint", degToRad(0)},
                {"left_hand_thumb_joint", degToRad(0)},
                {"right_torso_shoulder_mounting_joint", degToRad(65)},
                {"right_shoulder_mounting_shoulder_joint", degToRad(65)},
                {"right_shoulder_bicep_joint", degToRad(45)},
                {"right_bicep_forearm_joint", degToRad(-45)},
                {"right_forearm_wrist_joint", degToRad(0)},
                {"right_wrist_hand_joint", degToRad(45)},
                {"right_hand_thumb_joint", degToRad(0)},
                {"eyebrows_joint", degToRad(18)},
                {"mouth_top_joint", degToRad(0)},
                {"mouth_bottom_joint", degToRad(10)}
            };

            for( auto joints_it = joints_map.begin(); joints_it != joints_map.cend(); ++joints_it )
            {
                auto & joint = joints_it->second;

                double val = 0.;
                //use JointName class to convert from int id to string
                if (defaultPos.find(bandit::JointName(joints_it->first)) != defaultPos.end())
                    val = defaultPos.find(bandit::JointName(joints_it->first))->second;

                joint.setPos( val );
            }
            try
            {
                bandit_driver_.sendAllJointPos();
                break;
            }
            catch( std::runtime_error & e )
            {
                ROS_WARN( "Failed to set initial joint positions: %s", e.what() );
                std::this_thread::sleep_for( std::chrono::seconds( 1 ) );
            }
        }


        try
        {
            // This callback gets called whenever processPendingMessages
            // receives a valid state update from bandit
            bandit_driver_.registerStateCB( boost::bind( &BanditNode::stateCB, this, boost::ref( joints_pub_ ) ) );
            ROS_INFO( "registered state callback" );
            ros::spinOnce();

        }
        catch ( bandit::BanditException& e )
        {
            ROS_ERROR( "Caught bandit exception: %s\n", e.what() );
        }
    }

    bool paramCB( bandit_driver::Params::Request &req, bandit_driver::Params::Response &res )
    {
        res = param_response_;
        return true;
    }

    // This callback is invoked when we get a new joint command
    void jointCB( const bandit_driver::JointArrayConstPtr& joint_array )
    {
    	ROS_INFO("Joint Callback!");
        // Iterate through all joints in Joint Array
        std::vector<bandit_driver::Joint>::const_iterator joint_it = joint_array->joints.begin();
        for ( size_t i = 0; joint_it != joint_array->joints.end(); ++joint_it, ++i )
        {
            // Set the joint position
            ROS_INFO( "setting joint %u to angle: %f\n", joint_it->id, radToDeg( joint_it->angle ) );

            // Set the joint position; if this index doesn't exist, Bandit will throw an error so make sure to account for this
            bandit_driver_.setJointPos( (int16_t)joint_it->id, joint_it->angle );
        }
        // Push out positions to bandit

        bandit_driver_.sendAllJointPos();
    }

    void targetCB( const sensor_msgs::JointStateConstPtr & target_joint_state )
    {
    	ROS_INFO("Target Callback!");
        auto joint_name_it = target_joint_state->name.cbegin();
        auto joint_position_it = target_joint_state->position.cbegin();
        for ( ; joint_name_it != target_joint_state->name.cend(); ++joint_name_it, ++joint_position_it )
        {
            bandit::JointName joint_name( *joint_name_it );

            ROS_INFO( "setting joint %i to angle: %f\n", joint_name.id_, radToDeg( *joint_position_it ) );
            bandit_driver_.setJointPos( joint_name, *joint_position_it );
        }

        bandit_driver_.sendAllJointPos();
    }

    // This callback is invoked when we get new state from bandit
    void stateCB( diagnostic_updater::DiagnosedPublisher<sensor_msgs::JointState>& joint_state_pub )
    {
        bandit_state_initialized_ = true;

        auto const now = ros::Time::now();

        auto const num_joints = bandit_driver_.getNumJoints();
        auto const eyebrows_joint_index = bandit_driver_.getJointId( std::string( "eyebrows_joint" ) );
        sensor_msgs::JointState joint_state;

        // reserve memory now to make push_back operations less costly
        joint_state.name.reserve( num_joints );
        joint_state.position.reserve( num_joints );

        // set velocity and effort to 0
        joint_state.velocity.assign( num_joints, 0 );
        joint_state.effort.assign( num_joints, 0 );

        joint_state.header.stamp = now;
        joint_state.header.frame_id = "torso_link";

        std::map<std::string, double> joint_states_map;
        auto const & joints_map = bandit_driver_.getJoints();

        for( auto joints_it = joints_map.cbegin(); joints_it != joints_map.cend(); ++joints_it )
        {
            auto const & joint = joints_it->second;

            auto const joint_name = joint.name;
            // auto const joint_pos = ( joint.getPos() ); // need to normalizeEuler!!
            auto const joint_pos = normalizeEuler( joint.getPos() );
            // the eyebrows are really two joints as far as robot_state_publisher is concerned
            if ( joint.name == eyebrows_joint_index )
            {
                auto l_brow_pos = -joint_pos;
                auto r_brow_pos = -joint_pos;

                std::string l_brow_name( "head_left_brow_joint" );
                std::string r_brow_name( "head_right_brow_joint" );

                joint_state.name.push_back( l_brow_name );
                joint_state.position.push_back( l_brow_pos );
                joint_states_map[l_brow_name] = l_brow_pos;

                joint_state.name.push_back( r_brow_name );
                joint_state.position.push_back( r_brow_pos );
                joint_states_map[r_brow_name] = r_brow_pos;
            }
            else
            {
                joint_state.name.push_back( joint_name );
                joint_state.position.push_back( joint_pos );
                joint_states_map[joint_name] = joint_pos;
            }
        }
        //??? bandit_state_pub_ptr_->publishTransforms( joint_states_map, now, "/bandit" );

        // #if QUICKDEV_ROS_VERSION >= ROS_VERSION_ELECTRIC

        //         bandit_state_pub_ptr_->publishFixedTransforms( "/bandit" );
        // #endif

        joint_state_pub.publish( joint_state );
    }

    void spin()
    {
        // update loop
        while ( ros::ok() )
        {
            try
            {
                //std::cerr << "SPIN" << std::endl;
                diagnostic_.update();
                // Process any pending messages from bandit
                //ROS_INFO( "processIO: %ld", loop_rate.expectedCycleTime().toNSec());


                //bandit_driver_.processIO( QUICKDEV_GET_RUNABLE_POLICY()::getLoopRateSeconds() * 1000000 );
                bandit_driver_.processIO( 5000 );
                //bandit_driver_.processIO(loop_rate.expectedCycleTime().toNSec());
                //ROS_INFO( "processPackets" );
                bandit_driver_.processPackets();
                ros::spinOnce();
            }
            catch ( bandit::BanditException& e )
            {
                ROS_ERROR( "Caught bandit exception: %s\n", e.what() );
                std::cerr << "BANDIT EXCEPTION" << std::endl;
            }
        }
        std::cerr << "ROS NOT OK DONE!!!" << std::endl;
    }
};

int main( int argc, char** argv )
{
    ros::init( argc, argv, "bandit_driver" );

    //check_user_input(..);
    ros::NodeHandle nh;
    ros::Rate r(100);

    printf( "%f %f\n", degToRad( 90.0 ), radToDeg( 1.570796 ) );

    BanditNode bandit_node( nh );
    bandit_node.spin();

    return 0;
}

#endif /* BANDIT_NODE_H_ */
