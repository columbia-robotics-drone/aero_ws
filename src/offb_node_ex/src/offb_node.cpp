/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
/* mavros_msgs contains custom messages required to operate 
services and topics provided by the mavros package */
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;

/* callback to save the current state of the autopilot 
this allows us to check connection, arming, and offboard flags
we subscribe to the mavros state topic */
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    /* publishes the commanded local position */
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    /* service client that requests arming from the service server in mavros */ 
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    /* service client that requests mode from the service server in mavros */ 
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz because the px4 flight stack has
    // a timeout of 500ms between two offboard commands. if this timeout is exceeded, the 
    // commander will fall back to the last mode the vehicle was in before entering offboard mode

    // recommended to enter offboard mode from POSCTL (hover) mode
    ros::Rate rate(20.0);

    // wait for FCU connection
    /* before publishing anything, we wait for the connection to be established between mavros and
    the autopilot on the px4. this loop exits as soon as a heartbeat message is received

    heartbeat message = message that shows that a system is present and responding
    https://pixhawk.ethz.ch/mavlink/

    px4 autopilot is a RTOS providing a POSIX-style environment
    */
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    /* ENU (East North Up) coordinates
    mavros translates these to NED (North East Down) coordinates */
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // switch to offboard mode (http://wiki.ros.org/mavros/CustomModes#PX4_native_flight_stack)
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // arm the quad to allow it to fly
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            // space out the service calls by 5 seconds so as not to flood the autopilot
            // with these requests
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // in larger systems, it is often useful to create a new thread which will be in charge
        // of periodically publishing the setpoint
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}