//
// Created by zhangcheng on 2020/8/25.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <learning_communication/OffboardState.h>

geometry_msgs::PoseStamped pose;


learning_communication::OffboardState current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state.mode = msg->mode;
    current_state.armed = msg->armed;
    current_state.connected = msg->connected;
}

void position_cb(const learning_communication::OffboardState::ConstPtr& msg)
{
    current_state.position[0] = msg->position[0];
    current_state.position[1] = msg->position[1];
    current_state.position[2] = msg->position[2];
    ROS_INFO("current height order: [%f] [m]",current_state.position[2]);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"offboard");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Subscriber position_sub = nh.subscribe<learning_communication::OffboardState>("order_position",10,position_cb);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    ros::Rate rate(20.0);

    while (ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    for(int i=100;ros::ok() && i>0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD"; //设定模式

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value=true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()){
        if (current_state.mode != "OFFBOARD" && (ros::Time::now()-last_request>ros::Duration(5.0))){
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("offboard enabled");
            }
            last_request = ros::Time::now();
        } else{
            if (!current_state.armed && (ros::Time::now()-last_request>ros::Duration(5.0))){
                if (arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (pose.pose.position.x != current_state.position[0] || pose.pose.position.y != current_state.position[1] || pose.pose.position.z != current_state.position[2]){
            pose.pose.position.x = current_state.position[0];
            pose.pose.position.y = current_state.position[1];
            pose.pose.position.z = current_state.position[2];
        }
        ROS_INFO("command height: [%f] [m]",pose.pose.position.z);
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
