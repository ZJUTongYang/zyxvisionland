/**
 * 2018.4.7
 * This is a velocity test for odroid
 * There are two things must be sure before starting
 * First, we must subscribe camera data rather than gazebo camera data
 * Second, we must using manul offboard rather than auto offboard
 * By Zhengyuxin
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_position;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, &position_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped twist;
    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    double origin_x, origin_y, origin_z;
	origin_x = current_position.pose.position.x;
	origin_y = current_position.pose.position.y;
	origin_z = current_position.pose.position.z;
    ROS_INFO("%f   %f   %f oh yeah!",origin_x, origin_y, origin_z);
    pose.pose.position.x = origin_x + 0;
    pose.pose.position.y = origin_y + 0;
    pose.pose.position.z = origin_z + 2;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 1;
    pose.pose.orientation.w = 0;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        bool arm_flag=0;
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){        //(indigo)mode_sent-->success
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                    arm_flag = 1;
                }
                last_request = ros::Time::now();
            }
        }
        /*if(current_state.mode == "OFFBOARD")
		{
			ROS_INFO("OFFBOARD ENABLE");
			if(current_state.armed)
			{
				ROS_INFO("ARMED");
				arm_flag = 1;
			}
		}*/

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();

        if(arm_flag ==1)
            break;
    }

    ros::Time start_time = ros::Time::now();
    while(ros::ok()){
        if(ros::Time::now() - start_time > ros::Duration(10.0))
            break;
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    start_time = ros::Time::now();
    while(ros::ok()){
        /*if(ros::Time::now() - start_time < ros::Duration(2.0))
        {
            twist.twist.linear.x = 1;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 0;
            ROS_INFO("Go Forward");
        }else if(ros::Time::now() - start_time < ros::Duration(4.0))
        {
            twist.twist.linear.x = -1;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 0;
            ROS_INFO("Go backward");
        }else if(ros::Time::now() - start_time < ros::Duration(6.0))
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 1;
            twist.twist.linear.z = 0;
            ROS_INFO("Go Left");
        }else if(ros::Time::now() - start_time < ros::Duration(8.0))
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = -1;
            twist.twist.linear.z = 0;
            ROS_INFO("Go Right");
        }else if(ros::Time::now() - start_time < ros::Duration(10.0))
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 1;
            ROS_INFO("Go Up");
        }else if(ros::Time::now() - start_time < ros::Duration(12.0))
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = -1;
            ROS_INFO("Go Down");
        }else
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 0;
            ROS_INFO("Keep stable");
        }*/
        if(ros::Time::now() - start_time < ros::Duration(2.0))
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 1;
            ROS_INFO("Go Up");
        }else if(ros::Time::now() - start_time < ros::Duration(7.0))
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 0;
            ROS_INFO("Keep stable");
        }else if(ros::Time::now() - start_time < ros::Duration(9.0))
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = -1;
            ROS_INFO("Go down");
        }else if(ros::Time::now() - start_time < ros::Duration(15.0))
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 0;
            ROS_INFO("Keep stable");
        }else if(ros::Time::now() - start_time < ros::Duration(17.0))
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 1;
            ROS_INFO("Go Up");
        }else if(ros::Time::now() - start_time < ros::Duration(22.0))
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 0;
            ROS_INFO("Keep stable");
        }else if(ros::Time::now() - start_time < ros::Duration(24.0))
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = -1;
            ROS_INFO("Go down");
        }else
        {
            twist.twist.linear.x = 0;
            twist.twist.linear.y = 0;
            twist.twist.linear.z = 0;
            ROS_INFO("Keep stable");
        }
        local_vel_pub.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
