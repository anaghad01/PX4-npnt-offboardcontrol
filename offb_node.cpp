/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <eigen-3.3.8/Eigen/Dense>

using namespace Eigen;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
   MatrixXf M(6, 6);
   MatrixXf b(6, 3);
   MatrixXf a(6, 6);
   MatrixXf out(6, 3);
   MatrixXf outd(6, 3);
   int z_des = 5,x_des=3,y_des=5;
   double x,y,z; 
   int t0=0, tf=2, t=0;
   M << 1,    t0,  t0^2,    t0^3,    t0^4,    t0^5,
        0,    1,   2*t0,    3*t0^2,  4*t0^3,  5*t0^4,
        0,    0,   2,       6*t0,    12*t0^2, 20*t0^3,
        1,    tf,  tf^2,    tf^3,    tf^4,    tf^5,
        0,    1,   2*tf,    3*tf^2,  4*tf^3,  5*tf^4,
        0,    0,   2,       6*tf,    12*tf^2, 20*tf^3;
   b << 0,    0,  0,
        0,    0,  0,
        0,    0,  0,
        x_des, z_des,  y_des,
        0,    0,   0,
        0,    0,   0;
   a= M.inverse()*b;
   
/*********************************************************/
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.linear.z = 0.0;

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

    ros::Time last_request = ros::Time::now();
    ros::Time time_start = ros::Time::now();
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
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
	out.row(0)= a.row(0);
        out.row(1)= a.row(1)*((ros::Time::now()-time_start).toSec());
        out.row(2)=a.row(2)*pow((ros::Time::now()-time_start).toSec(),2);
        out.row(3)=a.row(3)*pow((ros::Time::now()-time_start).toSec(),3);
        out.row(4)=a.row(4)*pow((ros::Time::now()-time_start).toSec(),4);
        out.row(5)=a.row(5)*pow((ros::Time::now()-time_start).toSec(),5);

	outd.row(0)= a.row(0);
        outd.row(1)= a.row(1);
        outd.row(2)= 2*a.row(2)*((ros::Time::now()-time_start).toSec());
        outd.row(3)= 3*a.row(3)*pow((ros::Time::now()-time_start).toSec(),2);
        outd.row(4)= 4*a.row(4)*pow((ros::Time::now()-time_start).toSec(),3);
        outd.row(5)= 5*a.row(5)*pow((ros::Time::now()-time_start).toSec(),4);

	 /*vel.linear.x = outd.coeff(0,0);
         vel.linear.z = outd.coeff(0,1);
         vel.linear.y = outd.coeff(0,2);*/

         pose.pose.position.x = out.coeff(0,0);
         pose.pose.position.z = out.coeff(0,1);
         pose.pose.position.y = out.coeff(0,2);

	/*pose.pose.position.x = sin(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());
 pose.pose.position.y = cos(2.0*M_PI*0.1*(ros::Time::now()-time_start).toSec());
	pose.pose.position.x=0;
	pose.pose.position.y=0;
	pose.pose.position.z=0;
	/*vel.linear.x=0.5;
	vel.linear.y=0.5;
	vel.linear.z=0.5;*/	

	//local_vel_pub.publish(vel);
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
