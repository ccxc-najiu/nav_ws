# 可询问人员

刘威涛

# src

# A-LOAM-devel



# blue_lcm

该文件主要从topic `/cmd_vel`接收控制信息，通过回调函数`cmd_velHandle`将其转换后通过lcm发送到小车自带的驱动系统，再通过lcm接收由小车驱动系统返回的反馈信息，并publish到`/pose` topic

## topic

![image-20220311153340563](D:/实用工具/github/notes/res/image-20220311153340563.png)

## main

1.创建节点(blue_lcm_pub)

2.让节点订阅topic `/cmd_vel`，订阅回调函数为`cmd_velHandle`，通过该回调函数就可以发出控制信息，由小车自带的驱动系统接收

3.设置节点发布的话题`/pose`，消息类型`geometry_msgs::PoseStamped`，使用lcm_create创建`lcm_t *`对象，参数为`udpm://224.0.0.1:7667?ttl=1`，调用`robot_control_t_subscribe`函数，通过该函数将从小车驱动系统返回的参数值publish到`/pose` topic

### cmd_velHandle

订阅消息格式为`geometry_msgs::TwistStamped`

该回调函数主要调用了LCMSend2V()，并打印了订阅消息的内容，即直线速度与角速度，下面主要看LCMSend2V()

1.根据传递参数v, w 即`cmd_vel`里的`geometry_msgs::TwistStamped`中的`twist.linear.x*1000`与`twist.angular.z`，计算两轮差速，得到left，right，计算公式如下，它只有两个轮，靠差速转弯

这里应该是根据想要的小车整体速度v和角速度w，计算出分解的左右轮速度，但在函数里只计算出这两个值，而并没有使用这两个值，不知为何？

```c++
if (v == 0.0){
	left = - w * 500;
	right = w * 500;
}else{
	left = (v - w / 8 ) * 8000;
	right = (v + w / 8 ) * 8000;
}
```

2.将数据打包成data（打包的数据除了一些默认参数，就只有v, w，并没有之前计算的left, right），然后调用`robot_control_t_publish()`

3.在`robot_control_t_publish()`中将data数据编码，然后通过`lcm_publish`送出，通道名为`LCM_OUT_DOOR_BLUE_01`

补充：根据曾师兄所说，小车本身有一套驱动系统，它接受使用`lcm_publish`发出的`LCM_OUT_DOOR_BLUE_01`的消息，并依靠lcm返回`LCM_OUT_DOOR_BLUE_01`通道的信息，再被主函数接收到，调用回调函数`robot_control_handler_stub`

### robot_control_t_subscribe

该函数主要调用了`lcm_subscribe (lcm, channel,robot_control_t_handler_stub, n);`

所以直接看回调函数`robot_control_handler_stub`

这里主要调用了函数`user_hadnle`即`n->f`即`LCMRecvHandle`，下面直接看此函数，它接收了在`cmd_velHandle`中所发的信息，得到robotctrldata，它的类型和之前的data一致

1.switch robotctrldata->commandid

2.case 10，即命令id为10的情况下，打印一些东西

3.case 11，构造`geometry_msgs::PoseStamped`类型数据gpspose，通过`pubpose`发布，这里`pubpose`参考之前`main()`里的设置，发布到话题`/pose`，注意，这里已经是接收回的消息了。

### topic

![image-20220311153340563](D:/实用工具/github/notes/res/image-20220311153340563.png)



## lcm介绍

### lcm_create

```c++
lcm_t * lcm_create (const char *provider);
// lcm_create，lcm_t *的构造函数，其参数为null或一种特殊类型的字符串
// 如下"provider://network?option1=value1&option2=value2&...&optionN=valueN"

// provider	Initialization string specifying the LCM network provider. 
// If this is NULL, and the environment variable "LCM_DEFAULT_URL" is defined, then the environment variable is used instead. 
// If this is NULL and the environment variable is not defined, then default settings are used.
```

![image-20220310102021273](D:/实用工具/github/notes/res/image-20220310102021273.png)

![image-20220310102052008](D:/实用工具/github/notes/res/image-20220310102052008.png)

# data_generate

在该文件夹共生成三个node，它们都订阅topic `/move_base_simple/goal`与topic `/initialpose`，并分别生成相应处理后的信息publish到相应的topic

在BoundarypointGenerate里，它分别设置了pose.position.z=0 or 1，其余信息一致，publish到对应topic `boundarylow_trajectory`与topic `boundaryhigh_trajectory`

在topoGraph，它将所订阅话题输入的单数个msg视为线段起点，双数个msg视为线段终点，publish到topic `visualization_marker`，这里应当是用在nviz仿真当中

在waypointGenerate，它与BoundarypointGenerate类似，但没有设置pose.position.z，publish到对应topic `waypoint_trajectory`

## topic

![image-20220311143640121](D:/实用工具/github/notes/res/image-20220311143640121.png)

## topoGraph

这个文件主要从 topic `/move_base_simple/goal`获取信息msg，通过msg信息生成点集合topoMap，并制作线段发布到`visualization_marker`

### main
1.生成节点（topoGraph），得到topoGraph_file_dir参数（暂时不明该参数有何作用）

2.订阅topic `/move_base_simple/goal`，回调函数`generateTopoMap_callback`

3.订阅topic `/initialpose`，回调函数deletePoint_callback

4.publish topic `visualization_marker`, meesage type 为 `visualization_msgs::Marker`

### generateTopoMap_callback

以从topic订阅的msg为参数，调用generateTopoPoint(msg)，所以我们直接看generateTopoPoint()

订阅的消息类型为`geometry_msgs::PoseStamped::ConstPtr`

routePoint为结构体，firstPoint，secondPoint为指向routePoint类型的指针，topoMap为routePoint指针向量

```c++
struct  routePoint{
      float x;
      float y;
      float z;
      int id;
      vector<routePoint*> neighbor;
      vector<float> distance;
};
```

1.判断msg的位置是否超过1000，如果超过，保存目前所有拓扑点的信息，同时结束函数（不明意义，因为不知道所订阅的消息的实际意义）

2.如果msg位置不超过1000，将msg里的位置信息导入临时节点tempPoint(routePoint类型),遍历topoMap里所有的点，以3为界，只有tempPoint与topoMap里任意一点距离小于3，则将tempPoint与topoMap里的这一点视作同一点，取newPoint=topoMap里的这一点，否则，将newPont=tempPoint，然后topoMap.pushback(newPoint)，即无论如何，newPoint都指向topoMap里的一点（区别只在于topoMap有没有新增点）

---

*以上两个步骤都是从msg中获取信息，然后将信息存储在topoMap中，并且保证topoMap都是相隔距离大于3的点，即没有重复点*

---

3.totalNum是一个用来检测订阅函数促发次数的全局变量，也可以看作msg的总个数，这里如果totalNum为单，则取firstPoint=newPoint，如果totalNum为双，则取secondPoint=newPoint，并将这两个彼此视作邻居，用代码则表示为

```c++
float dis=sqrt((firstPoint->x-secondPoint->x)*(firstPoint->x-secondPoint->x)+(firstPoint->y-secondPoint->y)*(firstPoint->y-secondPoint->y));
firstPoint->neighbor.push_back(secondPoint);
firstPoint->distance.push_back(dis);
secondPoint->neighbor.push_back(firstPoint);
secondPoint->distance.push_back(dis);
```

4.取firstPoint与secondPoint，做成visualization_msgs::Marker形式，然后发布

---

*第3，4两个步骤主要提取msg里的信息为newPoint，然后视单数newPoint为线段起点，双数newPoint为线段终点，制作线段以visulization_msgs::Marker形式Publish，这里有问题在于它不在乎newPoint是topoMap之前已存在的点还是新增的点，有可能单数和双数的点是同一个点，线段也没有意义，不知为何要这么做*

---

### deletePoint_callback

它只调用了deleteWaypoint()，下面直接看这个函数

1.将topoMap清空

### topic

![image-20220311114846936](D:/实用工具/github/notes/res/image-20220311114846936.png)

## waypointGenerate

从topic `/move_base_simple/goal`与topic `/initialpose`里接收信息，如果是前者，将msg转化为Point3d类型添加到waypointSet中，然后将waypointSet所有数据打包，再加上由`tf::createQuaternionMsgFromRollPitchYaw(0, 90, 0);`生成的四元数形成`pathWaypoint`，发布到topic `waypoint_trajectory`。如果是后者，则将waypointSet最上的元素pop，然后打包剩下的waypointSet元素全体，再加上由`tf::createQuaternionMsgFromRollPitchYaw(0, 90, 0);`生成的四元数形成`pathWaypoint`，发布到topic `waypoint_trajectory`。

---

waypointSet： Point3d vector类型，

waypoint_file：打开参数`/waypointGenerate/waypoint_file_dir`里存储路径对应文件的句柄

waypoint_filr_dir：存放参数`/waypointGenerate/waypoint_file_dir`里的值

waypointpath_pub：节点发布到topic `waypoint_trajectory`的句柄

---

### main

1.生成node (waypointGenerate), 从Parameter Server得到名为`/waypointGenerate/waypoint_file_dir`的参数

2.将节点设置为订阅topic `/move_base_simple/goal`，消息类型为`geometry_msgs::PoseStamped`，句柄为`subWaypoint`，回调函数`getWaypoint_callback`。与此同时，将节点设置为订阅topic `/initialpose`，消息类型为`geometry_msgs::PoseWithCovarianceStamped`，句柄为`subdelete`，回调函数`deletePoint_callback`

3.告诉master该节点准备要发送`nav_msgs::Path`类型的消息到topic `waypoint_trajectory`

4.打开参数`/waypointGenerate/waypoint_file_dir`里存储的路径，准备写入，这里可以参考`waypoint_generate.launch`，找到这个参数，看到其路径为`$(find data_generate)/data/waypoints.ply`，即`roscd data_generate/data/waypoints.ply`

### getWaypoint_callback

该部分整体思路是从topic `/move_base_simple/goal`接收信息，经过一番转换后添加到vector集合`waypointSet`中，然后将这集合发布到topic `waypoint_trajectory`

---

这个回调函数只调用了getWaypoint(msg)，下面直接看这个函数

1.如果msg->pose.position.x或msg->pose.position.y大于1000，将`waypointSet`中的所有信息保存到参数`/waypointGenerate/waypoint_file_dir`里存储的路径中，然后函数结束

2.如果msg->pose.position.x or y不超过1000，再次判断，如果msg->pose.position.z小于0，函数结束

3.如果msg->pose.position.z大于0，将msg里的信息存储到tempPoint中，然后waypointSet.push_back(tempPoint)，之后，将waypointSet里的所有信息存放在pathWaypoint中，其中，它的`pose.orientation`四元数使用`tf::createQuaternionMsgFromRollPitchYaw(0, 90, 0)`生成

4.执行waypointpath_pub.publish(pathWaypoint)，即将pathWaypoint发布到topic `waypoint_trajectory`

### deletePoint_callback

该函数简单的清除了最新接收到的的位置信息，然后发布剩下的所有位置信息到topic `waypoint_trajectory`

---

该函数直接调用了deleteWaypoint()，下面直接看这个函数

1.如果waypointSet没有任何成员，直接返回

2.将waypointSet中最顶层的成员，即最新从topic `/move_base_simple/goal`得到的msg清除，然后如`getWaypoint_callback`一般将集合发布到topic `waypoint_trajectory`

### topic

![image-20220311114408914](D:/实用工具/github/notes/res/image-20220311114408914.png)

## boundaryGenerate

从topic `/move_base_simple/goal`与topic `/initialpose`获取数据，分别添加到vector `boundarypointSetlow`与vector `boundarypointSethigh`中，然后将两个vector的所有数据，再加上由`tf::createQuaternionMsgFromYaw(0);`所生成的四元数，其中`pose.position.z`分别设置为0，1，然后分别发布到topic `boundarylow_trajectory`与topic `boundaryhigh_trajectory`中

---

boundarypointSetlow: Point3d vector类型

boundarypointSethigh: Point3d vector类型

---

### main

1.生成节点`BoundarypointGenerate`，并从parameter server得到参数`/boundaryGenerate/boundary_file_dir`，并存储在`string`类型变量`boundary_file_dir`

2.设置节点，订阅`geometry_msgs::PoseStamped`消息类型话题`/move_base_simple/goal`与`geometry_msgs::PoseWithCovarianceStamped`消息类型话题，`/initialpose`，句柄分别为`subBoundarypointlow`与`subBoundarypointhigh`，回调函数分别为`getBoundarypointlow_callback`与`getBoundarypointhigh_callback`

3.设置节点，告诉master该节点将要向topic `boundarylow_trajectory`发布`nav_msgs::Path`类型的消息

4.设置节点，告诉master该节点将要向topic `boundaryhigh_trajectory`发布`nav_msgs::Path`类型的消息

5.打开参数`/boundaryGenerate/boundary_file_dir`里存储的路径，准备写入，关于参数值，可以查看`boundarypoint_generate.launch`文件，为`$(find data_generate)/data/boundary.ply`，即`roscd data_generate/data/boundary.ply`

### getBoundarypointlow_callback

此函数只调用了getBoundaryLowpoint(msg)，下面直接看这个函数

1.如果msg->pose.position.x or y 大于1000，则调用saveFile()，将vector boundarypointSetlow 与 vector boundarypointSethigh写入到参数`/boundaryGenerate/boundary_file_dir`所存储路径对应的文件中

2.如果msg->pose.position.x or y均小于1000，将msg.pose.position里的x、y、z信息打包生成Point3d类型数据tempPoint，然后boundarypointSetlow.push_back(tempPoint)添加到boundarypointSetlow中

3.将boundarypointSetlow所有数据，加上由`tf::createQuaternionMsgFromYaw(0);`生成的四元数打包成`nav_msgs::Path`类型数据pathWaypoint，**其中pose.position.z要被设置为0**，发布到topic `boundarylow_trajectory`

### getBoundarypointhigh_callback

该回调函数只调用了`getBoundaryhighpoint(msg)`，下面直接来看这个函数

1.如果msg->pose.position.x or y 大于1000，则调用saveFile()，将vector boundarypointSetlow 与 vector boundarypointSethigh写入到参数`/boundaryGenerate/boundary_file_dir`所存储路径对应的文件中

2.如果msg->pose.position.x or y均小于1000，将msg.pose.position里的x、y、z信息打包生成Point3d类型数据tempPoint，然后boundarypointSethigh.push_back(tempPoint)添加到boundarypointSethigh中

这里`tempPoint.z=1.0`，没有与getBoundarypointlow_callback对应，不知为何？

3.将boundarypointSethigh所有数据，加上由`tf::createQuaternionMsgFromYaw(0);`生成的四元数打包成`nav_msgs::Path`类型数据pathWaypoint，**其中pose.position.z要被设置为1**，发布到topic `boundaryhigh_trajectory`

### topic

![image-20220311142750672](D:/实用工具/github/notes/res/image-20220311142750672.png)

# ground_based_autonomy_basic-melodic

## terrain_analysis

三维激光地形分析，暂时未用到

## vehicle_simulator

车辆仿真

## velodyne_simulator

不明

## local_planner

## waypoint_example

### globalPlanner

规划部分，可结合data_generate/topoGraph一起看

---

graphMapDir: string类型变量

topoMap: routePoint* vector类型变量

#### main

1.设置string，放置存储地址，并读取该地址所对应的文件

这里可以略微改进，因为这里设置string时直接用了绝对地址，但可以把该地址在写在launch文件里，设置为`$(find data_generate)/data/topoMap.txt$`，然后再这里使用node.getparam("parameter_name", graphMapDir)即可。这样就只需要设置相对路径，也便于移植

2.读取graphMap第一行，即拓扑点的数量到topoMapSize，根据此将文件里对应的拓扑点push_back到routePoint* vector类型变量topoMap中

3.继续读取graphMap，补全topoMap中每一点的neighbor与distance

全三个步骤在于从txt文件中完全复制出topoMap

4.生成节点`waypointExample`，从parameter引入一大堆参数，下面直接看它的launch文件——`globalPlanner.launch`

```xml
<launch>
  <node pkg="waypoint_example" type="globalPlanner" name="globalPlanner" output="screen" required="true">
    <param name="waypoint_file_dir" type="string" value="$(find waypoint_example)/data/waypoints.ply" />
    <param name="boundary_file_dir" type="string" value="$(find waypoint_example)/data/boundary.ply" />
    <param name="waypointXYRadius" type="double" value="0.5" />
    <param name="waypointZBound" type="double" value="5.0" />
    <param name="waitTime" type="double" value="0" />
    <param name="frameRate" type="double" value="5.0" />
    <param name="speed" type="double" value="1.0" />
    <param name="sendSpeed" type="bool" value="true" />
    <param name="sendBoundary" type="bool" value="false" />
  </node>
</launch>
```

5.设置节点，订阅topic `/state_estimation`，消息类型`nav_msgs::Odometry`，回调函数`poseHandler`，句柄subPose

6.设置节点，发布到topic `/speed`，消息类型`std_msgs::Float32`，句柄为`pubSpeed`

7.设置节点，发布到topic`/navigation_boundary`, 消息类型`geometry_msgs::PolygonStamped`，句柄为pubBoundary

8.设置节点，发布到topic `way_point`，消息类型`geometery_msgs::PointStamped`，句柄为pubWaypoint

9.设置节点，订阅topic `/move_base_simple/goal`，消息类型`geometery_msgs::PoseStamped`，句柄为subWaypoint，回调函数`goal_callback`

10.设置节点，发布到topic `topoPoint`，消息类型`sensor_msgs::PointCloud2`，句柄为pubTopoPoint

11.设置节点，发布到topic `topoBigMap`，消息类型`sensor_msgs::PointCloud2`,句柄为topoMapTotal_pub

12.设置节点，发布到topic `topoMap`，消息类型`visulization_msgs::Marker`，句柄为topoMap_pub

以上5-12步完成了节点的发布与订阅设置

13.定义`visualization_msgs::Marker`类型数据line_list_bigMap, 并向其中填充信息，frame_id="map", id=2, ns="lines", action=ADD, pose.orientation.w=1.0, type=LINE_LIST, scale.x=0.8, color.r=1.0, color.a=1.0，然后以线段的方式向topoMap里填充点（对每一点，先push_back本身，再push_back neighbor）

14.if (sendBoundary)，这里是定义在本文件的全局变量，如果定义，则`readBoundaryFile()`，读取存储在变量`boundary_file_dir`, 由前面所说的参数读取步骤可知，该变量的值是存储在参数服务器上的参数`boundary_file_dir`，值为`$(find waypoint_example)/data/boundary.ply"`，即`roscd waypoint_example/data/boundary.ply`，读取其中数据，存储在全局变量`boundary`中

15.将boundary中的数据放进`geometry_msgs::PolygonStamped`类型全局变量`boundaryMsgs`中

16.设置ros::Rate rate(100)，进入while()循环

17.

#### poseHandle

此为订阅回调函数，订阅topic `/state_estimation`，接收消息类型为`nav_msgs::Odometry`，并把接收到的的消息存储在curTime, vehicleX, vehicleY, vehicleZ中

#### goal_callback

此为订阅回调函数，订阅topic `/move_base_simple/goal`, 消息类型`geometry_msgs::PoseStamped`



# velodyne-master

