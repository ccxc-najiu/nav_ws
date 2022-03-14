// created by lgt in 20210621
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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>


#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <iostream>
#include <fstream>
#include <set>
#include <map>
#include <queue>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

const double PI = 3.1415926;

string waypoint_file_dir;
string boundary_file_dir;
double waypointXYRadius = 0.5;
double waypointZBound = 5.0;
double waitTime = 0;
double waitTimeStart = 0;
bool isWaiting = false;
double frameRate = 5.0;
double speed = 1.0;
bool sendSpeed = true;
bool sendBoundary = true;

pcl::PointCloud<pcl::PointXYZ>::Ptr waypoints(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>());

float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
double curTime = 0, waypointTime = 0;

ros::Subscriber subPose;
ros::Subscriber subWaypoint;

ros::Publisher pubWaypoint;
ros::Publisher pubSpeed;
ros::Publisher pubTopoPoint ;

ros::Publisher topoMapTotal_pub;
ros::Publisher topoMap_pub;

ros::Publisher pubBoundary;
std_msgs::Float32 speedMsgs;

geometry_msgs::PolygonStamped boundaryMsgs;

geometry_msgs::PointStamped waypointMsgs;

Point2f startPoint, endPoint;

bool newGoal = false;
bool newPlan=false;







struct  routePoint{
   
      float x;
      float y;
      float z;
      int id;
      vector<routePoint*> neighbor;
      vector<float> distance;
      routePoint* parent;
};


routePoint* firstPoint=new routePoint();
routePoint* secondPoint=new routePoint();

vector<routePoint*> topoMap;
int totalNum=1;











struct graphNode
{
  graphNode(){

  };
  Point2f pointReal;
  vector<graphNode *> neighbor;
  graphNode *parent;
};
vector<graphNode *> graphNodeSet;

// bool bfs(graphNode *startNode, graphNode *endNode, vector<graphNode *> &path, map<graphNode *, int> &flag)
// {

//   queue<graphNode *> processNode;
//   processNode.push(startNode);

//   flag[startNode] = 1;

//   bool find = false;
//   graphNode *endNodeSearch;
//   while (!processNode.empty())
//   {

//     graphNode *node = processNode.front();
//     processNode.pop();
//     if (node == endNode)
//     {
//       find = true;
//       endNodeSearch = node;
//       break;
//     }

//     // flag[node] = 1;

//     int nodeNeigborSize = node->neighbor.size();

//     for (int i = 0; i < nodeNeigborSize; i++)
//     {
//       graphNode *temp = node->neighbor[i];
//       if (flag[temp] == 0)
//       {
//         temp->parent = node;
//         processNode.push(temp);
//         flag[temp] = 1;
//       }
//     }
//   }

//   if (find)
//   {
//     vector<graphNode *> pathInvese;
//     while (endNodeSearch != startNode)
//     {
//       pathInvese.push_back(endNodeSearch);
//       endNodeSearch = endNodeSearch->parent;
//     }
//     pathInvese.push_back(endNodeSearch);

//     int pathNuber = pathInvese.size();
//     // cout<<"path size is  "<<pathNuber<<endl;

//     for (int i = pathInvese.size() - 1; i >= 0; i--)
//     {
//       path.push_back(pathInvese[i]);
//     }
//   }
// };



bool bfs(routePoint *startNode, routePoint *endNode, vector<routePoint *> &path, map<routePoint *, int> &flag)
{

  queue<routePoint *> processNode;
  processNode.push(startNode);

  flag[startNode] = 1;

  bool find = false;
  routePoint *endNodeSearch;
  while (!processNode.empty())
  {

    routePoint *node = processNode.front();
    processNode.pop();
    if (node == endNode)
    {
      find = true;
      endNodeSearch = node;
      break;
    }

    // flag[node] = 1;

    int nodeNeigborSize = node->neighbor.size();

    for (int i = 0; i < nodeNeigborSize; i++)
    {
      routePoint *temp = node->neighbor[i];
      if (flag[temp] == 0)
      {
        temp->parent = node;
        processNode.push(temp);
        flag[temp] = 1;
      }
    }
  }

  if (find)
  {
    vector<routePoint* > pathInvese;
    while (endNodeSearch != startNode)
    {
      pathInvese.push_back(endNodeSearch);
      endNodeSearch = endNodeSearch->parent;
    }
    pathInvese.push_back(endNodeSearch);

    int pathNuber = pathInvese.size();
    // cout<<"path size is  "<<pathNuber<<endl;

    for (int i = pathInvese.size() - 1; i >= 0; i--)
    {
      path.push_back(pathInvese[i]);
    }
  }
};

// reading boundary from file function
//  从边界文件中读取边界点值（10个边界点，5个组成矩形的一周，十个构成一个体）
void readBoundaryFile()
{
  FILE *boundary_file = fopen(boundary_file_dir.c_str(), "r");
  if (boundary_file == NULL)
  {
    printf("\nCannot read input files, exit.\n\n");
    exit(1);
  }

  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header")
  {
    val = fscanf(boundary_file, "%s", str);
    if (val != 1)
    {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element")
    {
      val = fscanf(boundary_file, "%d", &pointNum);
      if (val != 1)
      {
        printf("\nError reading input files, exit.\n\n");
        exit(1);
      }
    }
  }

  boundary->clear();
  pcl::PointXYZ point;
  int val1, val2, val3;
  for (int i = 0; i < pointNum; i++)
  {
    val1 = fscanf(boundary_file, "%f", &point.x);
    val2 = fscanf(boundary_file, "%f", &point.y);
    val3 = fscanf(boundary_file, "%f", &point.z);

    if (val1 != 1 || val2 != 1 || val3 != 1)
    {
      printf("\nError reading input files, exit.\n\n");
      exit(1);
    }

    boundary->push_back(point);
  }

  fclose(boundary_file);
}

// vehicle pose callback function
void poseHandler(const nav_msgs::Odometry::ConstPtr &pose)
{
  curTime = pose->header.stamp.toSec();

  vehicleX = pose->pose.pose.position.x;
  vehicleY = pose->pose.pose.position.y;
  vehicleZ = pose->pose.pose.position.z;
}


// void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {

//   pcl::PointXYZ point;
//   if (msg->pose.position.z >= 0)
//   {

//     geometry_msgs::PoseStamped pt = *msg;
//     point.x = pt.pose.position.x;
//     point.y = pt.pose.position.y;
//     point.z = pt.pose.position.z;

//     startPoint.x = vehicleX, startPoint.y = vehicleY;
//     endPoint.x = point.x, endPoint.y = point.y;

//     float startMinScore = 1000;
//     float endMinScore = 1000;
//     graphNode *startNode, *endNode;

//     for (int i = 0; i < graphNodeSet.size(); i++)
//     {
//       float score = (graphNodeSet[i]->pointReal.x - startPoint.x) * (graphNodeSet[i]->pointReal.x - startPoint.x) + (graphNodeSet[i]->pointReal.y - startPoint.y) * (graphNodeSet[i]->pointReal.y - startPoint.y);
//       if (score < startMinScore)
//       {
//         startMinScore = score;
//         startNode = graphNodeSet[i];
//       }
//     }

//     for (int i = 0; i < graphNodeSet.size(); i++)
//     {
//       float score = (graphNodeSet[i]->pointReal.x - endPoint.x) * (graphNodeSet[i]->pointReal.x - endPoint.x) + (graphNodeSet[i]->pointReal.y - endPoint.y) * (graphNodeSet[i]->pointReal.y - endPoint.y);
//       if (score < endMinScore)
//       {
//         endMinScore = score;
//         endNode = graphNodeSet[i];
//       }
//     }

//     map<graphNode *, int> flag;
//     for (int i = 0; i < graphNodeSet.size(); i++)
//     {
//       flag[graphNodeSet[i]] = 0;
//     }

//     vector<graphNode *> path;
//     bfs(startNode, endNode, path, flag);

//     for (int i = 0; i < graphNodeSet.size(); i++)
//     {
//       flag[graphNodeSet[i]] = 0;
//     }

//     for (int i = 0; i < path.size(); i++)
//       cout << path[i]->pointReal << endl;

//     waypoints->clear();
//     waypoints->points.clear();

//     pcl::PointXYZ pointPath;
//     for (int i = 0; i < path.size(); i++)
//     {
//       pointPath.x = path[i]->pointReal.x;
//       pointPath.y = path[i]->pointReal.y;
//       pointPath.z = 0.0;
//       waypoints->points.push_back(pointPath);
//     }

//     pointPath.x = endPoint.x;
//     pointPath.y = endPoint.y;
//     pointPath.z = 0.0;
//     waypoints->points.push_back(pointPath);
//     cout << "sub goal point number has     " << waypoints->points.size() << endl;
//     newGoal = true;
//     newPlan=true;



//      pcl::PointCloud<pcl::PointXYZ> cloud;
//     // 添加点云数据
//     cloud.width = waypoints->points.size();
//     cloud.height=2;
//     cloud.points.resize(cloud.width * cloud.height);
//     for (size_t i = 0; i < cloud.points.size (); ++i)
//     {
//         cloud.points[i].x = waypoints->points[i].x;
//         cloud.points[i].y = waypoints->points[i].y;
//         cloud.points[i].z = waypoints->points[i].z;
//     }
//      cloud.is_dense = true;
//      sensor_msgs::PointCloud2 output;
//     //把点云转化为ros消息
//     pcl::toROSMsg(cloud, output);
//     output.header.frame_id = "/map";
//     pubTopoPoint.publish(output);

//   }
// }

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

  pcl::PointXYZ point;
  if (msg->pose.position.z >= 0)
  {

    geometry_msgs::PoseStamped pt = *msg;
    point.x = pt.pose.position.x;
    point.y = pt.pose.position.y;
    point.z = pt.pose.position.z;

    startPoint.x = vehicleX, startPoint.y = vehicleY;
    endPoint.x = point.x, endPoint.y = point.y;

    float startMinScore = 1000;
    float endMinScore = 1000;
    routePoint *startNode, *endNode;

    for (int i = 0; i < topoMap.size(); i++)
    {
      float score = (topoMap[i]->x - startPoint.x) * (topoMap[i]->x - startPoint.x) + (topoMap[i]->y - startPoint.y) * (topoMap[i]->y - startPoint.y);
      if (score < startMinScore)
      {
        startMinScore = score;
        startNode = topoMap[i];
      }
    }

    for (int i = 0; i < topoMap.size(); i++)
    {
      float score = (topoMap[i]->x - endPoint.x) * (topoMap[i]->x - endPoint.x) + (topoMap[i]->y - endPoint.y) * (topoMap[i]->y - endPoint.y);
      if (score < endMinScore)
      {
        endMinScore = score;
        endNode = topoMap[i];
      }
    }

    map<routePoint *, int> flag;
    for (int i = 0; i < topoMap.size(); i++)
    {
      flag[topoMap[i]] = 0;
    }

    vector<routePoint *> path;
    bfs(startNode, endNode, path, flag);

    for (int i = 0; i < topoMap.size(); i++)
    {
      flag[topoMap[i]] = 0;
    }

    // for (int i = 0; i < path.size(); i++)
    //   cout << path[i]->pointReal << endl;

   // waypoints->clear();
    waypoints->points.clear();

    pcl::PointXYZ pointPath;
    pointPath.x = startPoint.x;
    pointPath.y = startPoint.y;
    pointPath.z = 0.0;
    waypoints->points.push_back(pointPath);

    for (int i = 0; i < path.size(); i++)
    {
      pointPath.x = path[i]->x;
      pointPath.y = path[i]->y;
      pointPath.z = 0.0;
      waypoints->points.push_back(pointPath);
    }

    pointPath.x = endPoint.x;
    pointPath.y = endPoint.y;
    pointPath.z = 0.0;
    waypoints->points.push_back(pointPath);
    cout << "sub goal point number has     " << waypoints->points.size()-1 << endl;
    newGoal = true;
    newPlan=true;



     pcl::PointCloud<pcl::PointXYZ> cloud;
    // 添加点云数据
    cloud.width = waypoints->points.size();
    cloud.height=2;
    cloud.points.resize(cloud.width * cloud.height);
    for (size_t i = 1; i < cloud.points.size (); i++)
    {
        cloud.points[i].x = waypoints->points[i].x;
        cloud.points[i].y = waypoints->points[i].y;
        cloud.points[i].z = waypoints->points[i].z;
    }
     cloud.is_dense = true;
     sensor_msgs::PointCloud2 output;
    //把点云转化为ros消息
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = "/map";
    pubTopoPoint.publish(output);




    //  根据拓扑图，发布拓扑图信息，以线段束方式

          visualization_msgs::Marker line_list_topoMap;
          line_list_topoMap.header.frame_id = "map";
        //  line_list.header.stamp = ros::Time::now();
          line_list_topoMap.ns = "lines";
          line_list_topoMap.action = visualization_msgs::Marker::ADD;
          line_list_topoMap.pose.orientation.w = 1.0;
          line_list_topoMap.id = 2;
          line_list_topoMap.type = visualization_msgs::Marker::LINE_LIST;

          line_list_topoMap.scale.x = 0.8;

          line_list_topoMap.color.r = 1.0;
          line_list_topoMap.color.a = 1.0;

          for(int i=0;i<waypoints->points.size()-1;i++){
              //routePoint* fir=topoMap[i];
              pcl::PointXYZ pointFir,pointSec;
              pointFir=waypoints->points[i];
              pointSec=waypoints->points[i+1];
             
                  geometry_msgs::Point pFir,pSec;
                  pFir.x = pointFir.x;
                  pFir.y = pointFir.y;
                  pFir.z = pointFir.z;
                  line_list_topoMap.points.push_back(pFir);

                  pSec.x = pointSec.x;
                  pSec.y = pointSec.y;
                  pSec.z = pointSec.z;
                  line_list_topoMap.points.push_back(pSec);
              }
          topoMap_pub.publish(line_list_topoMap);




  }
}
// 获取机器人当前位置，确定此时需要发布的子目标点、速度及边界信息
int main(int argc, char **argv)
{

  string graphMapDir = "/home/siasun/nav_ws/src/data_generate/data/topoMap.txt";
  ifstream graphMap(graphMapDir);

  // 读取数据，构建图
  cout<<" read topo map "<<endl;
  int topoMapSize;
  graphMap>>topoMapSize;
  cout<<"topo point size is "<<topoMapSize<<endl;
  for(int i=0;i<topoMapSize;i++){
      routePoint* newPoint=new routePoint;
      
      graphMap>>newPoint->id;
      graphMap>>newPoint->x;
      graphMap>>newPoint->y;
      graphMap>>newPoint->z;
      cout<<" id "<<newPoint->id<<" x "<<newPoint->x<<" y "<<newPoint->y<<" z "<<newPoint->z<<endl;
      topoMap.push_back(newPoint);
  }
  for(int i=0;i<topoMapSize;i++){
      int idNum;
      graphMap>>idNum;
      int idSize;
      graphMap>>idSize;

      cout<<"topoMap id "<<idNum<<" has neighbor ";

      for(int j=0;j<idSize;j++){
         int idNeigh;
         graphMap>>idNeigh;
         topoMap[idNum]->neighbor.push_back(topoMap[idNeigh]);
         cout<<" "<<idNeigh<<" ";
      }  
       cout<<endl;
       cout<<"topoMap id "<<idNum<<" has neighbor distance ";


       for(int j=0;j<idSize;j++){
         double disNeigh;
         graphMap>>disNeigh;
         topoMap[idNum]->distance.push_back(disNeigh);
                  cout<<" "<<disNeigh<<" ";

      } 
      cout<<endl;
  }







  // vector<Point2f> topoPointSet;
  // int graphNodeNum;
  // graphMap >> graphNodeNum;
  // for (int i = 0; i < graphNodeNum; i++)
  // {
  //   Point2f temp;
  //   float tempX;
  //   float tempY;
  //   graphMap >> tempX;
  //   graphMap >> tempY;
  //   temp.x = tempX;
  //   temp.y = tempY;
  //   topoPointSet.push_back(temp);
  //   // cout << temp << endl;
  // }

  // // 构建图

  // for (int i = 0; i < topoPointSet.size(); i++)
  // {
  //   Point2f temp = topoPointSet[i];
  //   graphNode *gNode = new graphNode();
  //   gNode->pointReal = temp;
  //   graphNodeSet.push_back(gNode);
  // }

  // for (int i = 0; i < graphNodeNum; i++)
  // {
  //   Point2f temp;
  //   float tempX;
  //   float tempY;
  //   graphMap >> tempX;
  //   graphMap >> tempY;
  //   temp.x = tempX;
  //   temp.y = tempY;

  //   graphNode *nodeFirst;
  //   for (int j = 0; j < graphNodeNum; j++)
  //   {
  //     if (graphNodeSet[j]->pointReal == temp)
  //       nodeFirst = graphNodeSet[j];
  //   }

  //   int edgeSize;
  //   graphMap >> edgeSize;

  //   for (int j = 0; j < edgeSize; j++)
  //   {

  //     graphNode *nodeSecond;
  //     Point2f tempFirst, tempSecond;

  //     float tempX;
  //     float tempY;
  //     graphMap >> tempX;
  //     graphMap >> tempY;
  //     tempSecond.x = tempX;
  //     tempSecond.y = tempY;

  //     for (int h = 0; h < graphNodeNum; h++)
  //     {

  //       if (graphNodeSet[h]->pointReal == tempSecond)
  //         nodeSecond = graphNodeSet[h];
  //     }
  //     nodeFirst->neighbor.push_back(nodeSecond);
  //   }

  //}

  ros::init(argc, argv, "waypointExample");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("waypoint_file_dir", waypoint_file_dir);
  nhPrivate.getParam("boundary_file_dir", boundary_file_dir);
  nhPrivate.getParam("waypointXYRadius", waypointXYRadius);
  nhPrivate.getParam("waypointZBound", waypointZBound);
  nhPrivate.getParam("waitTime", waitTime);
  nhPrivate.getParam("frameRate", frameRate);
  nhPrivate.getParam("speed", speed);
  nhPrivate.getParam("sendSpeed", sendSpeed);
  nhPrivate.getParam("sendBoundary", sendBoundary);

  boundaryMsgs.header.frame_id = "/map";

  waypointMsgs.header.frame_id = "/map";

  subPose = nh.subscribe<nav_msgs::Odometry>("/state_estimation", 5, poseHandler);

  pubSpeed = nh.advertise<std_msgs::Float32>("/speed", 5);

  pubBoundary = nh.advertise<geometry_msgs::PolygonStamped>("/navigation_boundary", 5);

  pubWaypoint = nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);

  subWaypoint = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, goal_callback);

  pubTopoPoint = nh.advertise<sensor_msgs::PointCloud2>("/topoPoint", 100);

  topoMapTotal_pub = nh.advertise<visualization_msgs::Marker>("topoBigMap", 10);

  topoMap_pub = nh.advertise<visualization_msgs::Marker>("topoMap", 10);




 //  根据拓扑图，发布拓扑图信息，以线段束方式

          visualization_msgs::Marker line_list_bigMap;
          line_list_bigMap.header.frame_id = "map";
        //  line_list.header.stamp = ros::Time::now();
          line_list_bigMap.ns = "lines";
          line_list_bigMap.action = visualization_msgs::Marker::ADD;
          line_list_bigMap.pose.orientation.w = 1.0;
          line_list_bigMap.id = 2;
          line_list_bigMap.type = visualization_msgs::Marker::LINE_LIST;

          line_list_bigMap.scale.x = 0.8;

          line_list_bigMap.color.r = 1.0;
          line_list_bigMap.color.a = 1.0;

          for(int i=0;i<topoMap.size();i++){
              routePoint* fir=topoMap[i];
              for(int j=0;j<fir->neighbor.size();j++){
                  geometry_msgs::Point pFir,pSec;
                  pFir.x = fir->x;
                  pFir.y = fir->y;
                  pFir.z = fir->z;
                  line_list_bigMap.points.push_back(pFir);

                  pSec.x = fir->neighbor[j]->x;
                  pSec.y = fir->neighbor[j]->y;
                  pSec.z = fir->neighbor[j]->z;
                  line_list_bigMap.points.push_back(pSec);
              }
          }

  // read waypoints from file
  //readWaypointFile();

  if (sendBoundary)
  {
    readBoundaryFile();

    int boundarySize = boundary->points.size();
    boundaryMsgs.polygon.points.resize(boundarySize);
    for (int i = 0; i < boundarySize; i++)
    {
      boundaryMsgs.polygon.points[i].x = boundary->points[i].x;
      boundaryMsgs.polygon.points[i].y = boundary->points[i].y;
      boundaryMsgs.polygon.points[i].z = boundary->points[i].z;
    }
  }

  ros::Rate rate(100);
  bool status = ros::ok();

  int wayPointID = 0;

  while (status)
  {
    ros::spinOnce();

    if (newGoal)
    {
      if(newPlan){
        wayPointID=0;
        newPlan=false;
      }
      

      float disX = vehicleX - waypoints->points[wayPointID].x;
      float disY = vehicleY - waypoints->points[wayPointID].y;
      float disZ = vehicleZ - waypoints->points[wayPointID].z;

      float dis = sqrt(disX * disX + disY * disY);
      // cout << "distance is    " << dis << endl;
      // cout << vehicleX << "  " << vehicleY << endl;
      // cout << waypoints->points[wayPointID].x << "  " << waypoints->points[wayPointID].y << endl;

      // start waiting if the current waypoint is reached
      //到达子目标点，等一会儿再发布下一个子目标点
      if (sqrt(disX * disX + disY * disY) < waypointXYRadius)
      {
        wayPointID++;
        cout << wayPointID << endl;
        if (wayPointID >= waypoints->points.size())
        {
          
          cout << wayPointID << endl;
          wayPointID=0;
          newGoal=false;

        }
      }

      // publish waypoint, speed, and boundary messages at certain frame rate
      // 以一定频率发布目标点、速度、边界信息
      if (curTime - waypointTime > 1.0 / frameRate)
      {

        {
          waypointMsgs.header.stamp = ros::Time().fromSec(curTime);
          waypointMsgs.point.x = waypoints->points[wayPointID].x;
          waypointMsgs.point.y = waypoints->points[wayPointID].y;
          waypointMsgs.point.z = waypoints->points[wayPointID].z;
          pubWaypoint.publish(waypointMsgs);
        }

        if (sendSpeed)
        {
          speedMsgs.data = speed;
          pubSpeed.publish(speedMsgs);
        }

        if (sendBoundary)
        {
          boundaryMsgs.header.stamp = ros::Time().fromSec(curTime);
          pubBoundary.publish(boundaryMsgs);
        }

        waypointTime = curTime;
      }
    }

    //  pcl::PointCloud<pcl::PointXYZ> cloud;
    // // 添加点云数据
    // cloud.width = waypoints->points.size();
    // cloud.height=2;
    // cloud.points.resize(cloud.width * cloud.height);
    // for (size_t i = 0; i < cloud.points.size (); ++i)
    // {
    //     cloud.points[i].x = waypoints->points[i].x;
    //     cloud.points[i].y = waypoints->points[i].y;
    //     cloud.points[i].z = waypoints->points[i].z;
    // }
    //  cloud.is_dense = true;
    //  sensor_msgs::PointCloud2 output;
    // //把点云转化为ros消息
    // pcl::toROSMsg(cloud, output);
    // output.header.frame_id = "/camera";
    // pubTopoPoint.publish(output);

    topoMapTotal_pub.publish(line_list_bigMap);
    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
