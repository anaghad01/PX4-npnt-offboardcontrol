
#include <ros/ros.h>
#include <mavros_msgs/SetValue.h>//custom msg created
#include <mavros/mavros_plugin.h>
#include <mavros_msgs/Param.h>
#include <mavros_msgs/ParamValue.h>

//Changes parameter value after message is published
void subCallback(const mavros_msgs::SetValue& msg)
{ 

        ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<mavros_msgs::Param>("mavros/Param", 10);
 	mavros_msgs::Param pu;

     	pu.param_id="MPC_LAND_SPEED";
     	pu.value.integer=2;

        pub.publish(pu);//'rostopic echo mavros/Param' to see published values

 	ROS_INFO("Value set");
}


int main(int argc, char **argv)
{ 
    ros::init(argc, argv, "sv2");
    ros::NodeHandle nh;

    ros::Publisher local_pub = nh.advertise<mavros_msgs::SetValue>
            ("mavros/SetValue", 10);
    ros::Subscriber sub = nh.subscribe("mavros/SetValue", 10, subCallback);
    ros::Publisher pub = nh.advertise<mavros_msgs::Param>
            ("mavros/Param", 10);
    
    ros::Rate rate(20.0);

    int count =0;
    while(ros::ok())
    {
     	mavros_msgs::SetValue msg;
     	
        msg.setpoint= 3;

        local_pub.publish(msg);

        ros::spinOnce(); 
        rate.sleep();

        ++count;
    }

    return 0;
}
