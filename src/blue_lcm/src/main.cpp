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

// #define PI 3.14159265

// lcm_create，lcm_t *的构造函数，其参数为null或一种特殊类型的字符串
// 如下"provider://network?option1=value1&option2=value2&...&optionN=valueN"
lcm_t *lcm;
ros::Publisher pubpose;
// bool subok = true;

void *lcmrecvfunc(void *param)
{
    while(1)
    {
	  lcm_handle(lcm);
    }
}
//左右两轮差速 V1=V+W*L/2     V2=V-W*L/2
void LCMSend2V(double v,double w){
    // printf("\r\n[%s] \n", __func__);
    robot_control_t data;
    int left, right;

    if (v == 0.0){
	  left = - w * 500;
	  right = w * 500;
    }else{
	  left = (v - w / 8 ) * 8000;
	  right = (v + w / 8 ) * 8000;
    }
    
    
    std::cout << "leftspeed:  " << left << "rightspeed:   " << right << std::endl;
    
    double d_params[2];
    d_params[0] = v;
    d_params[1] = w;      
    data.commandid = 20;
    data.nsparams = 0;
    data.nbparams = 0;
    data.ndparams = 2;
    data.dparams = d_params;
    data.niparams = 0;
    robot_control_t_publish(lcm, "LCM_OUT_DOOR_BLUE_01", &data);
}

// void LCMSend2V(int left, int right){
// 	// printf("\r\n[%s] \n", __func__);
// 	robot_control_t data;
// 	double d_params[2];
// 	d_params[0] = left;
// 	d_params[1] = right;
// 	data.commandid = 20;
// 	data.nsparams = 0;
// 	data.nbparams = 0;
// 	data.ndparams = 2;
// 	data.dparams = d_params;
// 	data.niparams = 0;
// 	robot_control_t_publish(lcm, "LCM_OUT_DOOR_BLUE_01", &data);
// }

void LCMRecvHandle( const lcm_recv_buf_t *rbuf,const char * channel, const robot_control_t *robotctrldata,void *user)
{


    switch (robotctrldata->commandid)
    {
    case 10:{
	  printf("\r\n[%s] 码盘 左 %.0f 右 %.0f", __func__, robotctrldata->dparams[0], robotctrldata->dparams[1]);
	  break;
    }
    case 11:{
	  
// 	      if ( subok == true)
	  LCMSend2V(0.0, 0.0);
	  
	  printf("\r\n[%s] GPS QGPS: %d QHEAD: %d | x: %.0f y: %.0f t: %.0f", __func__
							  , robotctrldata->iparams[0]
							  , robotctrldata->iparams[1]
							  , robotctrldata->dparams[0]
							  , robotctrldata->dparams[1]
							  , robotctrldata->dparams[2]);
	  geometry_msgs::PoseStamped gpspose;
	  gpspose.header.frame_id = "/velodyne";
	  gpspose.header.stamp = ros::Time::now();
	  gpspose.pose.position.x = robotctrldata->dparams[0];
	  gpspose.pose.position.y = robotctrldata->dparams[1];
	  gpspose.pose.position.z = 0.0;
	  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
					  (0.0, 0.0, robotctrldata->dparams[2]);
	  gpspose.pose.orientation.w = geoQuat.w;
	  gpspose.pose.orientation.x = geoQuat.x;
	  gpspose.pose.orientation.y = geoQuat.y;
	  gpspose.pose.orientation.z = geoQuat.z;
	  pubpose.publish(gpspose);
	  
	  break;
    }
    default:
	  break;
    }
    
// 	subok = true;
    return ;
}

void cmd_velHandle(const geometry_msgs::TwistStampedConstPtr& cmdmsg)
{
//     subok = false;
    LCMSend2V(cmdmsg->twist.linear.x*1000, cmdmsg->twist.angular.z);
    std::cout << cmdmsg->twist.linear.x << "    " << cmdmsg->twist.angular.z << std::endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "blue_lcm_pub"); //blue_lcm_pub为node名称，但会被blue_lcm.launch的name覆盖
    ros::NodeHandle n("public_namespace");
    ros::NodeHandle np("~private_namespace");
    ros::Subscriber cmd_sub = np.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 100, &cmd_velHandle);
    pubpose = n.advertise<geometry_msgs::PoseStamped>("/pose", 100);
    pthread_t lcm_thread;
    lcm = lcm_create("udpm://224.0.0.1:7667?ttl=1");

    robot_control_t_subscribe(lcm,"LCM_OUT_DOOR_BLUE_01",&LCMRecvHandle,NULL);
    pthread_create(&lcm_thread,NULL,&lcmrecvfunc,NULL);
    pthread_detach(lcm_thread);

    // 测试
    // 	while(1)
    // 	{
    // 		LCMSend2V(100, 100);
    // 		usleep(100000);
    // 	}
    ros::spin();
    return 0;
}
