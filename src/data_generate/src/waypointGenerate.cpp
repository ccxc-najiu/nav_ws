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

ros::Publisher waypointpath_pub;

string waypoint_file_dir;
FILE *waypoint_file;

vector<Point3d> waypointSet;

void saveFile()
{
    cout << "save file start    ......          " << endl;
    //  string str1 = "ply";
    fprintf(waypoint_file, "%s", "ply\n");
    fprintf(waypoint_file, "%s", "format ascii 1.0\n");
    fprintf(waypoint_file, "%s", "element vertex ");
    int num = waypointSet.size() * 5;
    fprintf(waypoint_file, "%d", num);
    fprintf(waypoint_file, "%s", "\n");

    fprintf(waypoint_file, "%s", "property float x\n");
    fprintf(waypoint_file, "%s", "property float y\n");
    fprintf(waypoint_file, "%s", "property float z\n");
    fprintf(waypoint_file, "%s", "end_header\n");
    for (int j = 0; j < 5; j++)
    {

        for (int i = 0; i < waypointSet.size(); i++)
        {
            fprintf(waypoint_file, "%f", waypointSet[i].x);
            fprintf(waypoint_file, "%s", "\t");
            fprintf(waypoint_file, "%f", waypointSet[i].y);
            fprintf(waypoint_file, "%s", "\t");
            fprintf(waypoint_file, "%f", waypointSet[i].z);
            fprintf(waypoint_file, "%s", "\n");
        }
    }

    fclose(waypoint_file);
    cout << "save file end    ......          " << endl;
}

void getWaypoint(const geometry_msgs::PoseStamped::ConstPtr &msg)
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
            waypointSet.push_back(tempPoint);

            for (int i = 0; i < waypointSet.size(); i++)
                cout << "waypoint  number  " << i << "  point  is  " << waypointSet[i].x << "   " << waypointSet[i].y << "   " << waypointSet[i].z << endl;

            nav_msgs::Path pathWaypoint;
            pathWaypoint.header.frame_id = "/camera_init";
            for (int i = 0; i < waypointSet.size(); i++)
            {
                geometry_msgs::PoseStamped this_pose_stamped;
                this_pose_stamped.pose.position.x = waypointSet[i].x;
                this_pose_stamped.pose.position.y = waypointSet[i].y;

                //geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(1.57);
                 geometry_msgs::Quaternion goal_quat=tf::createQuaternionMsgFromRollPitchYaw(0, 90, 0);
                this_pose_stamped.pose.orientation.x = goal_quat.x;
                this_pose_stamped.pose.orientation.y = goal_quat.y;
                this_pose_stamped.pose.orientation.z = goal_quat.z;
                this_pose_stamped.pose.orientation.w = goal_quat.w;
                cout<<goal_quat.w<<" "<<goal_quat.x<<" "<<goal_quat.y<<" "<<goal_quat.z<<endl;

                ros::Time current_time;
                current_time = ros::Time::now();
                this_pose_stamped.header.stamp = current_time;
                this_pose_stamped.header.frame_id = "/camera_init";
                pathWaypoint.poses.push_back(this_pose_stamped);
            }

            waypointpath_pub.publish(pathWaypoint);
        }
    }
}

void deleteWaypoint()
{

    if (waypointSet.size() > 0)
    {

        waypointSet.pop_back();

        for (int i = 0; i < waypointSet.size(); i++)
            cout << "number  " << i << "  point  is  " << waypointSet[i].x << "   " << waypointSet[i].y << "   " << waypointSet[i].z << endl;
        if (waypointSet.size() > 0)
        {
            nav_msgs::Path pathWaypoint;
            pathWaypoint.header.frame_id = "/camera_init";
            for (int i = 0; i < waypointSet.size(); i++)
            {
                geometry_msgs::PoseStamped this_pose_stamped;
                this_pose_stamped.pose.position.x = waypointSet[i].x;
                this_pose_stamped.pose.position.y = waypointSet[i].y;

                geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);
                this_pose_stamped.pose.orientation.x = goal_quat.x;
                this_pose_stamped.pose.orientation.y = goal_quat.y;
                this_pose_stamped.pose.orientation.z = goal_quat.z;
                this_pose_stamped.pose.orientation.w = goal_quat.w;

                ros::Time current_time;
                current_time = ros::Time::now();
                this_pose_stamped.header.stamp = current_time;
                this_pose_stamped.header.frame_id = "/camera_init";
                pathWaypoint.poses.push_back(this_pose_stamped);
            }

            waypointpath_pub.publish(pathWaypoint);
        }
    }
    else
        cout << "pointset number  is  0" << endl;
}

void getWaypoint_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    getWaypoint(msg);
}

void deletePoint_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    deleteWaypoint();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "waypointGenerate");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("waypoint_file_dir", waypoint_file_dir);

    ros::Subscriber subWaypoint = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, getWaypoint_callback);
    ros::Subscriber subdelete = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100, deletePoint_callback);

    waypointpath_pub = nh.advertise<nav_msgs::Path>("waypoint_trajectory", 1, true);

    waypoint_file = fopen(waypoint_file_dir.c_str(), "w");

    ros::spin();

    return 0;
}
