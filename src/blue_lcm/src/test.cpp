#include <ros/ros.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <math.h>
#include "lcm/robot_control_t.h"
//#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test");
    ros::NodeHandle n;
    ros::Publisher pubpose = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 100);

    ros::Rate loop_rate(10);
    int count = 0;
    while(ros::ok())
    {
        geometry_msgs::TwistStamped msg;
        msg.header.seq = 0;
        msg.header.frame_id = "cwx";
        msg.twist.angular.z = 0;
        if (count < 10) {
            msg.twist.linear.x = 0.1;
        }
        else {
            msg.twist.linear.x = -0.1;
        }

        count++;
        if (count >= 20) {
            count = 0;
        }
        pubpose.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}
