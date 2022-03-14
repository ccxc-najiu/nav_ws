#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>

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

#include <visualization_msgs/Marker.h>

using namespace std;
using namespace cv;


ros::Publisher topoMap_pub;

string topoGraph_file_dir;
FILE *topoGraph_file;
//ofstream* graphMap;

int idNow=0;
float disThresh=3;

struct  routePoint{
   
      float x;
      float y;
      float z;
      int id;
      vector<routePoint*> neighbor;
      vector<float> distance;
};


routePoint* firstPoint=new routePoint();
routePoint* secondPoint=new routePoint();

vector<routePoint*> topoMap;
int totalNum=1;


void saveFile()
{
    cout << "save file start    ......          " << endl;


  // 写入数据
 
  ofstream graphMap(topoGraph_file_dir);
  graphMap<<topoMap.size()<<endl;
  for(int i=0;i<topoMap.size();i++){
      graphMap<<topoMap[i]->id<<" ";
      graphMap<<topoMap[i]->x<<" ";
      graphMap<<topoMap[i]->y<<" ";
      graphMap<<topoMap[i]->z<<" ";
      graphMap<<endl;
   }
   for(int i=0;i<topoMap.size();i++){
      graphMap<<topoMap[i]->id<<" ";
      graphMap<<topoMap[i]->neighbor.size()<<" ";
      for(int j=0;j<topoMap[i]->neighbor.size();j++){
          graphMap<<topoMap[i]->neighbor[j]->id<<" ";
      }
     for(int j=0;j<topoMap[i]->neighbor.size();j++){
          graphMap<<topoMap[i]->distance[j]<<" ";
      }
      graphMap<<endl;
  }




//   vector<Point2f> topoPointSet;
//   int graphNodeNum;
//   graphMap >> graphNodeNum;
//   for (int i = 0; i < graphNodeNum; i++)
//   {
//     Point2f temp;
//     float tempX;
//     float tempY;
//     graphMap >> tempX;
//     graphMap >> tempY;
//     temp.x = tempX;
//     temp.y = tempY;
//     topoPointSet.push_back(temp);
//     // cout << temp << endl;
//   }

//   // 构建图

//   for (int i = 0; i < topoPointSet.size(); i++)
//   {
//     Point2f temp = topoPointSet[i];
//     graphNode *gNode = new graphNode();
//     gNode->pointReal = temp;
//     graphNodeSet.push_back(gNode);
//   }

//   for (int i = 0; i < graphNodeNum; i++)
//   {
//     Point2f temp;
//     float tempX;
//     float tempY;
//     graphMap >> tempX;
//     graphMap >> tempY;
//     temp.x = tempX;
//     temp.y = tempY;

//     graphNode *nodeFirst;
//     for (int j = 0; j < graphNodeNum; j++)
//     {
//       if (graphNodeSet[j]->pointReal == temp)
//         nodeFirst = graphNodeSet[j];
//     }

//     int edgeSize;
//     graphMap >> edgeSize;

//     for (int j = 0; j < edgeSize; j++)
//     {

//       graphNode *nodeSecond;
//       Point2f tempFirst, tempSecond;

//       float tempX;
//       float tempY;
//       graphMap >> tempX;
//       graphMap >> tempY;
//       tempSecond.x = tempX;
//       tempSecond.y = tempY;

//       for (int h = 0; h < graphNodeNum; h++)
//       {

//         if (graphNodeSet[h]->pointReal == tempSecond)
//           nodeSecond = graphNodeSet[h];
//       }
//       nodeFirst->neighbor.push_back(nodeSecond);
//     }

//   }



    cout << "save file end    ......          " << endl;
}

void generateTopoPoint(const geometry_msgs::PoseStamped::ConstPtr &msg)
{

    if (abs(msg->pose.position.x) > 1000 || abs(msg->pose.position.y) > 1000){
        // 保存拓扑地图
          saveFile();
    }
      
    else
    {

        routePoint* tempPoint=new routePoint();
        tempPoint->x =msg->pose.position.x;
        tempPoint->y =msg->pose.position.y;
        tempPoint->z =msg->pose.position.z;

      
     
        //  遍历已有拓扑点，发现是否有同类点，如果有同类点，给出此点的指针地址
        int topoPointSize=topoMap.size();
        routePoint* newPoint=NULL;

        for(int i=0;i<topoPointSize;i++){
            float dis=sqrt((tempPoint->x-topoMap[i]->x)*(tempPoint->x-topoMap[i]->x)+(tempPoint->y-topoMap[i]->y)*(tempPoint->y-topoMap[i]->y));
            if(dis<disThresh){
                newPoint=topoMap[i];
                break;
            }
        }

        //  如果是未知点，新构建一个点，赋值，并给出指针地址
        if(newPoint==NULL){
            newPoint=new routePoint();
            newPoint->x=tempPoint->x;
            newPoint->y=tempPoint->y;
            newPoint->z=tempPoint->z;
            newPoint->id=idNow;
            idNow++;
            topoMap.push_back(newPoint);
            std::cout<<"new point generate"<<std::endl;
        }
        // 如果是已知点，也在newPoint上了
        
        delete tempPoint;
        //  检查当前点是线段的第一点，还是第二点
        if(totalNum%2==1)
        //  如果是第一点，记录他
            {
               firstPoint=newPoint;
               totalNum++;
            }
            else
        //  如果是第二点，构建第一点和第二点的链接关系，加入拓扑图中
            {
               secondPoint=newPoint;
               totalNum++;
               float dis=sqrt((firstPoint->x-secondPoint->x)*(firstPoint->x-secondPoint->x)+(firstPoint->y-secondPoint->y)*(firstPoint->y-secondPoint->y));
               firstPoint->neighbor.push_back(secondPoint);
               firstPoint->distance.push_back(dis);
               secondPoint->neighbor.push_back(firstPoint);
               secondPoint->distance.push_back(dis);
               std::cout<<"new edge generate"<<std::endl;


            }

        //  根据拓扑图，发布拓扑图信息，以线段束方式

          visualization_msgs::Marker line_list;
          line_list.header.frame_id = "camera_init";
         // line_list.header.frame_id = "map";
        //  line_list.header.stamp = ros::Time::now();
          line_list.ns = "lines";
          line_list.action = visualization_msgs::Marker::ADD;
          line_list.pose.orientation.w = 1.0;
          line_list.id = 2;
          line_list.type = visualization_msgs::Marker::LINE_LIST;

          line_list.scale.x = 0.8;

          line_list.color.r = 1.0;
          line_list.color.a = 1.0;

          for(int i=0;i<topoMap.size();i++){
              routePoint* fir=topoMap[i];
              for(int j=0;j<fir->neighbor.size();j++){
                  geometry_msgs::Point pFir,pSec;
                  pFir.x = fir->x;
                  pFir.y = fir->y;
                  pFir.z = fir->z;
                  line_list.points.push_back(pFir);

                  pSec.x = fir->neighbor[j]->x;
                  pSec.y = fir->neighbor[j]->y;
                  pSec.z = fir->neighbor[j]->z;
                  line_list.points.push_back(pSec);
              }
          }


          topoMap_pub.publish(line_list);


    }
}

void deleteWaypoint()
{
    totalNum=1;
    topoMap.clear();
    idNow=0;

  
}

void generateTopoMap_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    generateTopoPoint(msg);
}

void deletePoint_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    deleteWaypoint();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topoGraph");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("topoGraph_file_dir", topoGraph_file_dir);
    // 订阅goal，拓扑点生成
    ros::Subscriber subWaypoint = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, generateTopoMap_callback);
   // ros::Subscriber subWaypoint = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 10, generateTopoMap_callback);

    // 订阅poseinital，重新新建拓扑点
    ros::Subscriber subdelete = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 100, deletePoint_callback);
    // 广播拓扑地图线段
    topoMap_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

   // 该文件用以保存地图
   // topoGraph_file = fopen(topoGraph_file_dir.c_str(), "w");
   
    

    ros::spin();

    return 0;
}
