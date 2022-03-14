#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>

using namespace std;
using namespace cv;

ros::Publisher boundarypathlow_pub;
ros::Publisher boundarypathhigh_pub;

string boundary_file_dir;
FILE *boundary_file;
vector<Point3d> boundarypointSetlow;
vector<Point3d> boundarypointSethigh;

void saveFile()
{
	cout << "save file start    ......          " << endl;
	//  string str1 = "ply";
	fprintf(boundary_file, "%s", "ply\n");
	fprintf(boundary_file, "%s", "format ascii 1.0\n");
	fprintf(boundary_file, "%s", "element vertex ");
	int num = boundarypointSetlow.size()+boundarypointSethigh.size();
	num = num ;
	fprintf(boundary_file, "%d", num);
	fprintf(boundary_file, "%s", "\n");

	fprintf(boundary_file, "%s", "property float x\n");
	fprintf(boundary_file, "%s", "property float y\n");
	fprintf(boundary_file, "%s", "property float z\n");
	fprintf(boundary_file, "%s", "end_header\n");
	//for (int j = 0; j < 5; j++)
	{
		for (int i = 0; i < boundarypointSetlow.size(); i++)
		{
			fprintf(boundary_file, "%f", boundarypointSetlow[i].x);
			fprintf(boundary_file, "%s", "\t");
			fprintf(boundary_file, "%f", boundarypointSetlow[i].y);
			fprintf(boundary_file, "%s", "\t");
			fprintf(boundary_file, "%f", boundarypointSetlow[i].z);
			fprintf(boundary_file, "%s", "\n");

		}
		for (int i = 0; i < boundarypointSethigh.size(); i++)
		{
			fprintf(boundary_file, "%f", boundarypointSethigh[i].x);
			fprintf(boundary_file, "%s", "\t");
			fprintf(boundary_file, "%f", boundarypointSethigh[i].y);
			fprintf(boundary_file, "%s", "\t");
			fprintf(boundary_file, "%f", boundarypointSethigh[i].z);
			fprintf(boundary_file, "%s", "\n");
		}
	}

	fclose(boundary_file);
	cout << "save file end    ......          " << endl;
}

void getBoundaryLowpoint(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

	if (abs(msg->pose.position.x) > 1000 || abs(msg->pose.position.y) > 1000)
		   saveFile();
	else
	{

		if (msg->pose.position.z >= 0)
		{
			Point3d tempPoint;
			geometry_msgs::PoseStamped pt = *msg;
			tempPoint.x = pt.pose.position.x;
			tempPoint.y = pt.pose.position.y;
			tempPoint.z = pt.pose.position.z;
			boundarypointSetlow.push_back(tempPoint);

			for (int i = 0; i < boundarypointSetlow.size(); i++)
				cout << "boundary low  point set   number  " << i << "  point  is  " << boundarypointSetlow[i].x << "   " << boundarypointSetlow[i].y << "   " << boundarypointSetlow[i].z << endl;

			nav_msgs::Path pathBoundary;
			pathBoundary.header.frame_id = "/camera_init";
			for (int i = 0; i < boundarypointSetlow.size(); i++)
			{
				geometry_msgs::PoseStamped this_pose_stamped;
				this_pose_stamped.pose.position.x = boundarypointSetlow[i].x;
				this_pose_stamped.pose.position.y = boundarypointSetlow[i].y;
				this_pose_stamped.pose.position.z = 0;

				geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);
				this_pose_stamped.pose.orientation.x = goal_quat.x;
				this_pose_stamped.pose.orientation.y = goal_quat.y;
				this_pose_stamped.pose.orientation.z = goal_quat.z;
				this_pose_stamped.pose.orientation.w = goal_quat.w;

				ros::Time current_time;
				current_time = ros::Time::now();
				this_pose_stamped.header.stamp = current_time;
				this_pose_stamped.header.frame_id = "/camera_init";
				pathBoundary.poses.push_back(this_pose_stamped);
			}

			boundarypathlow_pub.publish(pathBoundary);
		}
	}
}


void getBoundaryhighpoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{

	if (abs(msg->pose.pose.position.x) > 1000 || abs(msg->pose.pose.position.y) > 1000)
		   saveFile();
	else
	{

		if (msg->pose.pose.position.z >= 0)
		{
			Point3d tempPoint;
			geometry_msgs::PoseWithCovarianceStamped pt = *msg;
			tempPoint.x = pt.pose.pose.position.x;
			tempPoint.y = pt.pose.pose.position.y;
			tempPoint.z = 1.0;
			boundarypointSethigh.push_back(tempPoint);

			for (int i = 0; i < boundarypointSethigh.size(); i++)
				cout << "boundary high  point set   number  " << i << "  point  is  " << boundarypointSethigh[i].x << "   " << boundarypointSethigh[i].y << "   " << boundarypointSethigh[i].z << endl;

			nav_msgs::Path pathBoundary;
			pathBoundary.header.frame_id = "/camera_init";
			for (int i = 0; i < boundarypointSethigh.size(); i++)
			{
				geometry_msgs::PoseStamped this_pose_stamped;
				this_pose_stamped.pose.position.x = boundarypointSethigh[i].x;
				this_pose_stamped.pose.position.y = boundarypointSethigh[i].y;
				this_pose_stamped.pose.position.z = 1.0;

				geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);
				this_pose_stamped.pose.orientation.x = goal_quat.x;
				this_pose_stamped.pose.orientation.y = goal_quat.y;
				this_pose_stamped.pose.orientation.z = goal_quat.z;
				this_pose_stamped.pose.orientation.w = goal_quat.w;

				ros::Time current_time;
				current_time = ros::Time::now();
				this_pose_stamped.header.stamp = current_time;
				this_pose_stamped.header.frame_id = "/camera_init";
				pathBoundary.poses.push_back(this_pose_stamped);
			}

			boundarypathhigh_pub.publish(pathBoundary);
		}
	}
}

// void deleteBoundarypoint()
// {

// 	if (boundarypointSet.size() > 0)
// 	{

// 		boundarypointSet.pop_back();

// 		for (int i = 0; i < boundarypointSet.size(); i++)
// 			cout << "number  " << i << "  point  is  " << boundarypointSet[i].x << "   " << boundarypointSet[i].y << "   " << boundarypointSet[i].z << endl;
// 		if (boundarypointSet.size() > 0)
// 		{
// 			nav_msgs::Path pathBoundary;
// 			pathBoundary.header.frame_id = "/camera_init";

// 			for (int i = 0; i < boundarypointSet.size(); i++)
// 			{
// 				geometry_msgs::PoseStamped this_pose_stamped;
// 				this_pose_stamped.pose.position.x = boundarypointSet[i].x;
// 				this_pose_stamped.pose.position.y = boundarypointSet[i].y;

// 				geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);
// 				this_pose_stamped.pose.orientation.x = goal_quat.x;
// 				this_pose_stamped.pose.orientation.y = goal_quat.y;
// 				this_pose_stamped.pose.orientation.z = goal_quat.z;
// 				this_pose_stamped.pose.orientation.w = goal_quat.w;

// 				ros::Time current_time;
// 				current_time = ros::Time::now();
// 				this_pose_stamped.header.stamp = current_time;
// 				this_pose_stamped.header.frame_id = "/camera_init";
// 				pathBoundary.poses.push_back(this_pose_stamped);
// 			}

// 			boundarypath_pub.publish(pathBoundary);
// 		}
// 	}
// 	else
// 		cout << "pointset number  is  0" << endl;
// }

void getBoundarypointlow_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	getBoundaryLowpoint(msg);
}

void getBoundarypointhigh_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
	getBoundaryhighpoint(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "BoundarypointGenerate");
	ros::NodeHandle nh;
	ros::NodeHandle nhPrivate = ros::NodeHandle("~");

	nhPrivate.getParam("boundary_file_dir", boundary_file_dir);

	ros::Subscriber subBoundarypointlow = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, getBoundarypointlow_callback);
	ros::Subscriber subBoundarypointhigh = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100, getBoundarypointhigh_callback);
	boundarypathlow_pub = nh.advertise<nav_msgs::Path>("boundarylow_trajectory", 1, true);
	boundarypathhigh_pub = nh.advertise<nav_msgs::Path>("boundaryhigh_trajectory", 1, true);

	boundary_file = fopen(boundary_file_dir.c_str(), "w");

	ros::spin();

	return 0;
}