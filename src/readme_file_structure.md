# 可询问人员

刘威涛

# src

# A-LOAM-devel



# blue_lcm

该文件主要从topic `/cmd_vel`接收控制信息，通过回调函数`cmd_velHandle`将其转换后通过lcm发送到小车自带的驱动系统，再通过lcm接收由小车驱动系统返回的反馈信息，并publish到`/pose` topic

## topic

![image-20220311153340563](https://gitee.com/ccxc1001/image/raw/master/readme_window_image/image-20220311153340563.png)

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

![image-20220311153340563](https://gitee.com/ccxc1001/image/raw/master/readme_window_image/image-20220311153340563.png)



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

![image-20220310102021273](https://gitee.com/ccxc1001/image/raw/master/readme_window_image/image-20220310102021273.png)

![image-20220310102052008](https://gitee.com/ccxc1001/image/raw/master/readme_window_image/image-20220310102052008.png)

# data_generate

在该文件夹共生成三个node，它们都订阅topic `/move_base_simple/goal`与topic `/initialpose`，并分别生成相应处理后的信息publish到相应的topic

在BoundarypointGenerate里，它分别设置了pose.position.z=0 or 1，其余信息一致，publish到对应topic `boundarylow_trajectory`与topic `boundaryhigh_trajectory`

在topoGraph，它将所订阅话题输入的单数个msg视为线段起点，双数个msg视为线段终点，publish到topic `visualization_marker`，这里应当是用在nviz仿真当中

在waypointGenerate，它与BoundarypointGenerate类似，但没有设置pose.position.z，publish到对应topic `waypoint_trajectory`

## topic

![image-20220311143640121](https://gitee.com/ccxc1001/image/raw/master/readme_window_image/image-20220311143640121.png)

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

![image-20220311114846936](https://gitee.com/ccxc1001/image/raw/master/readme_window_image/image-20220311114846936.png)

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

![image-20220311114408914](https://gitee.com/ccxc1001/image/raw/master/readme_window_image/image-20220311114408914.png)

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

![image-20220311142750672](https://gitee.com/ccxc1001/image/raw/master/readme_window_image/image-20220311142750672.png)

# ground_based_autonomy_basic-melodic

## terrain_analysis

三维激光地形分析，暂时未用到

## vehicle_simulator

车辆仿真

该ros包主要功能是实现了一个双轮差动底盘的模拟,接收cmd_vel速度信息,自己手写的机器人运动学微分方程来对机器人位姿进行推算,从而输出里程计odometry,而gazebo只是用来做模型显示.该包中包含了CMU制作的几个探索gazebo world,分别包含了不同的环境类型.

思维导图如下(点击调整到xmind格式)，它接收topic `/cmd_vel`，`/terrain_map`传回的传感器信息，进行机器人位置，姿态计算，发布到topic `/state_estimation`与`/gaozabo/set_model_state`

[![image-20220321094905319](https://gitee.com/ccxc1001/image/raw/master/readme_window_image/image-20220321094905319.png)](./xmind_project/ground_based_autonomy_basic-melodic/vehicle_simulator/vehicleSimulator.xmind)

---

全局变量

---

由于是仿真，所以并没有实际的传感器，很多的值都是估算而出，不必太过考究

---

odomTime: ros::Time类型，无默认值，在while循环中被调用`odomTime = ros::Time::now();`，为当前时间

stackNum: const int类型，设置为400，堆栈数，配合odomSendIDPointer与odomRecIDPointer使用，构成vehicleXStack, vehicleYStack, vehicleZStack, vehicleRollStack, float vehiclePitchStack, vehicleYawStack, terrainRollStack, terrainPitchStack, odomTimeStack

odomSendIDPointer: int类型，默认值为-1，在while循环中被调用，`odomSendIDPointer = (odomSendIDPointer + 1) % stackNum`，每一次循环加1，但不超过stackNum，表示发送信息的次数

odomRecIDPointer: int类型，默认值为0，在回调函数scanHandler被引用，`odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;`，表示接收点云信息的次数

vehicleYaw: float类型，默认值为0，在while循环中被调用 `vehicleYaw += 0.005 * vehicleYawRate;`，为当前偏航角

vehicleRoll: float类型，默认值为0，在while循环中被引用赋值，计算公式为

```c++
vehicleRoll = terrainRoll * cos(vehicleYaw) + terrainPitch * sin(vehicleYaw)
```

为小车翻滚角

vehiclePitch: float类型，默认值为0，在while循环中被引用赋值，计算公式为

```cpp
vehiclePitch = -terrainRoll * sin(vehicleYaw) + terrainPitch * cos(vehicleYaw);
```

为小车俯仰角

terrainRoll: float类型，默认值为0，在回调函数terrainCloudHandler中被引用赋值，应当为地形翻滚角

terrainPitch: float类型，默认值为0，在回调函数terrainCloudHandler中被引用赋值，应当为地形俯仰角

vehicleYawRate: float类型，默认值为0，在回调函数speedHandler被调用，接收topic `/cmd_vel`传递而来的信息

vehicleSpeed: float类型，默认值为0，在回调函数speedHandler被调用，接收topic `/cmd_vel`传递而来的信息

由vehicleYawRate与vehicleSpeed可以模拟计算出许多有用的信息

vehicleX: float类型，默认值为0，在while循环中由公式

```cpp
vehicleX += 0.005 * cos(vehicleYaw) * vehicleSpeed 
              + 0.005 * vehicleYawRate * (-sin(vehicleYaw) * sensorOffsetX - cos(vehicleYaw) * sensorOffsetY);
```

计算而出，为小车当前位置信息

vehicleY: float类型，默认值为0，在while循环中由公式

```cpp
vehicleY += 0.005 * sin(vehicleYaw) * vehicleSpeed 
              + 0.005 * vehicleYawRate * (cos(vehicleYaw) * sensorOffsetX - sin(vehicleYaw) * sensorOffsetY);
```

计算而出

vehicleZ: float类型，默认值为0，在while循环中由公式

```cpp
vehicleZ = terrainZ + vehicleHeight;
```

计算而出

下面解释其计算原理，之前有过设置，ros::rate(200)，所以一次循环的时间差不多差不多5ms（这里不严谨，因为执行回调函数也需要不定的时间，如果把ros.sleep放在三个位置计算之前就是标准的5ms了）

而vehicleYaw为偏航角，vehicleSpeed为线速度，`0.005 * cos(vehicleYaw) * vehicleSpeed `表示由线速度对小车X坐标的影响

vehicleYawRate为角速度，sensorOffsetX为传感器距中心在X方向的偏移，sensorOffsetY为传感器距中心在Y方向的偏移。由于小车传感器并不在小车中心，所以角速度也会产生影响，它的影响在后半部分`0.005 * vehicleYawRate * (-sin(vehicleYaw) * sensorOffsetX - cos(vehicleYaw) * sensorOffsetY)`，可以画图来辅助思考

vehicleY也是类似vehicleX计算

vehicleZ则取决于当前地形高度



---

该文件夹里面只有一个cpp文件vehicleSimulator.cpp，所以直接看它就好了

### main

1.初始化节点，命名为vehicleSimulator, 但vehicle_simulator.launch文件里，它的名字为vehicleSimulator，覆盖了之前的名字（覆盖了寂寞），通过getParam获取一堆参数，它们定义在vehicle_simulator.launch文件里

2.设置节点，订阅topic `/velodyne_points`，回调函数scanHandler，句柄subScan

3.设置节点，订阅topic `/terrain_map`，回调函数terrainCloudHandler，句柄subTerrainCloud

4.设置节点，订阅topic `/cmd_vel`，回调函数speedHandler，句柄subSpeed

5.设置节点，准备发布到topic `/state_estimation`，句柄pubVehicleOdom，message类型为`nav_msgs::Odometry`. 定义nav_msgs::Odometry类型变量odomData。设置其 head.frame_id="/map", child_frame_id="/sensor"

这里可以与waypoint_example文件夹下三个cpp文件联系起来，它们都订阅了这个topic，用以获得小车当前位置信息

6.设置节点，准备发布到topic `/gazebo/set_model_state`，句柄pubModelState，message type为`gazebo_msgs::Mode1State`，定义该类型变量cameraState, lidarState, 并设置其model_name分别为camera, lidar.

7.设置节点，准备发布到topic `/registered_scan`，句柄为pubScan，message type `sensor_msgs::PointCloud2`

8.设置ros::rate，频率为200Hz，进入循环，执行spinOnce()，执行订阅回调函数。

9.在循环内根据角速度vehicleYawRate, 地形地貌terrainPitch, terrainRoll来更新vehicleRoll, vehiclePitch, vehicleYaw，这里vehicleYaw要限制在-PI到PI内

10.根据小车当前偏航角vehicleYaw, 当前线速度vehicleSpeed, 当前角速度vehicleYawRate，传感器偏心距sensorOffsetX, sensorOffsetY，地形高度terrainZ, 小车中心高vehicleHeight来计算出vehicleX, vehicleY，计算方式参考全局变量处描述

11.更新时间，并把小车位置信息，时间，姿态角，地形姿态存储在堆栈中

12.定义`geometry_msgs::Quaternion`类型信息geoQuat，此为四元数，用以表示由vehicleRoll, vehiclePitch, vehicleYaw表示的旋转

13.向全局变量，`nav_msgs::Odeometry`类型数据odomData填充时间，四元数geoQuat，当前位置信息，三个方向角速度，线速度等信息，然后Publish到topic `/state_estimation`

14.将小车位置信息与姿态存储到cameraState与lidarState，然后发布到topic `/gazebo/set_model_state`

### scanHandler

这是传感器回调函数，应当是接收激光雷达传回的点云

此处从topic `/velodyne_points`接收点云，并将其转换到世界系下发布到topic `/registered_scan`

---

1.如果系统没有初始化，则是在执行五次回调函数后进行初始化，然后程序才能继续往下运行，这里应当是为了让传感器数据更加稳定

2.用一个while循环确保当前odomRecIDPointer指向的时间大于接收点云信息的时间，如果不大于且odomRecIDPointer小于odomSendIDPointer，则odomRecIDPointer+1，继续比较，如果时间大于或者odomRecIDPointer不能再加(再加就要超过odomSendIDPointer，这在逻辑上不可能，因为总是要线发送后接收)跳出循环，继续程序

总之就是确保在逻辑上时间的正确（实际情况不用考虑，仿真情形则要小心）

3.如果use_gazebo_time为true，则接收时间，接收时小车位置，姿态，地形姿态等信息使用接收时刻所对应的值，即odomRecIDPointer所指向的堆栈值，如果use_gazebo_time为false，则以上信息使用当前时刻的值，即在while循环中更新的最新值

4.清空全局变量scanData, 用它来接收所订阅topic 传来的点云数据，并去除异常值

5.将点云数据转化到世界坐标系下，转化公式尚不能理解

6.定义sensor_msgs::PointCloud2类型数据，设置head.stamp为接收时间，header.frame_id为`/map`，发布到topic `/registered_scan`

### terrainCloudHandler

---

这里也是点云类型数据，不过scanHandler传回来的是小车本身的数据，这里传回来的是地形地貌数据

根据数据计算出terrainPitch与terrainRoll，用以确定结合vehicleYawRate，vehicleYaw来计算出小车本身姿态角vehicleRoll，vehiclePitch与地形高度  terrainZ

具体转化过程尚不能理解

### speedHandler

接收topic `/cmd_vel`传过来的控制信息，设置vehicleSpeed的值为目标速度，vehicleYawRate的值为目标角速度，由于是在仿真环境，设定速度，角速度后可以直接达到，所以vehicleSpeed, vehicleYawRate也可以认定为由传感器获得的当前线速度，角速度

### topic



## velodyne_simulator

不明

## local_planner

### localPlanner

### pathFollower



## waypoint_example

### globalPlanner

该部分主要实现以下功能，订阅topic `/state_estimation`获得小车当前点信息，订阅topic /move_base_simple/goal获得终点信息，读取文件topoMap.txt得到拓扑地图信息，读取文件boundary.ply获得边界信息

在topic  `/move_base_simple/goal`回调函数内，根据拓扑地图信息与当前点，终点计算出一条从当前点到终点的拓扑路径waypoints，将其转化为点云发布到topic `/topoPoint`, 将其转为线段发布到topic `/topoMap`

在main函数while循环中，计算小车当前点与拓扑路径点的距离，确定当前目标点，发布当前目标到topic `/way_point`，同时发布速度到topic `/speed`，发布边界到`/navigation_boundary`，发布整个拓扑地图信息到topic `topoBigMap`

[流程图](./xmind_project/ground_based_autonomy_basic-melodic/waypoint_example/globalPlanner/globalPlanner.xmind)

---

规划部分，可结合data_generate/topoGraph一起看

---

- 全局变量

graphMapDir: string类型变量

topoMap: routePoint* vector类型变量

startPoint: Point2f类型，在goal_callback中用到，作为起始点，数据来源于全局变量vehicleX, vehicleY

endPoint: Point2f类型，在goal_callback中用到，作为终点，数据来源于订阅topic `/move_base_simple/goal`的输入信息，此即目标点

vehicleX: float类型，来源于所订阅topic `/state_estimation`所输入的信息，此为小车当前所在位置值

vehicleY: float类型，来源于所订阅topic `/state_estimation`所输入的信息，此为小车当前所在位置值

vehicleZ: float类型，来源于所订阅topic `/state_estimation`所输入的信息，此为小车当前所在位置值

find: bool类型，用于bfs中，判断是否有一条从startPoint到endPoint的拓扑路径

newGoal: bool类型，默认值为false, 在回调函数goal_callback中被引用，即从topic `/move_base_simple/goal`获取信息msg, if(msg->pose.position.z >= 0), 则newGoal=true

newPlan: bool类型，默认值为false, 在回调函数goal_callback中被引用，即从topic `/move_base_simple/goal`获取信息msg, if(msg->pose.position.z >= 0), 则newPlan = true

在goal_callback中，主要从订阅信息得到endPoint信息，而startPoint则由vehicleX, vehicleY给的，在这里函数主要用于寻找一条从startPoint到endPoint的拓扑路径，这里newGoal=true, newPlan=true可以理解

waypointXYRadius：全局变量，判定值，如果小车位置与目标点距离小于该值，则认定小车已经到了该点

curTime: float类型，默认值为0,在回调函数中`poseHandler`中被引用，将topic `/state_estimation` 所订阅信息的时间赋值给curTime，其记录当前时间

waypointTime: float类型，默认值为0，其记录在while循环中发布目标点信息的时间，用于与curTime进行比较

frameRate: double类型，用于设置在while循环中发布目标点信息的频率

sendSpeed: bool类型，用于在while循环中发布目标点信息时判断是否发布速度信息

sendBoundary: bool类型，用于在while循环中发布目标点信息时判断是否发布边界信息

speed: double类型，这是在while循环中发布速度信息时的速度

#### main

---

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

这里line_list_bigMap所用信息为topoMap，这表明它是不变的数据

14.if (sendBoundary)，这里是定义在本文件的全局变量，如果定义，则`readBoundaryFile()`，读取存储在变量`boundary_file_dir`, 由前面所说的参数读取步骤可知，该变量的值是存储在参数服务器上的参数`boundary_file_dir`，值为`$(find waypoint_example)/data/boundary.ply"`，即`roscd waypoint_example/data/boundary.ply`，读取其中数据，存储在全局变量`boundary`中

15.将boundary中的数据放进`geometry_msgs::PolygonStamped`类型全局变量`boundaryMsgs`中

16.设置ros::Rate rate(100)，进入while()循环，循环内主要让小车当前位置vehicleX, vehicleY与在回调函数`goal_callback`中所计算出的路径waypoints中的点作比较，如果小于waypointXYRadius，此为全局变量，则认为小车已经到达这一个点，从而驶向下一个点

如果行驶过程中`goal_callback`再次被调用，则说明有新的目标点，根据现在的位置生成新的路径waypoints，设置小车从新追踪新路径的第一个点，然后如上

如果行驶到了终点，则不再进行最终

这一切的实现是依据newGoal, newPlan两个全局变量实现的

17.在第16步中只描述了如何比较，已经转换目标点，并没有说明如何将目标点信息发出去，接下来给出相关信息。依旧是在while循环中，这里依据全局变量curTime, waypointTime和frameRate，最终实现以一定频率将目标点位置，当前时间也就是curTime发布到topic `way_point`

18.如果全局变量sendSpeed设置为true，则发布速度全局变量speed到topic `/speed`

19.如果全局变量seendBoundary设置为true, 则发布边界信息到topic `/navigation_boundary`

20.无论如何，在while循环中一定会发布line_list_big_Map即topoMap的全部信息而非仅仅到下一个目标点的路径，到topic `topoBigMap`

#### poseHandle

此为订阅回调函数，订阅topic `/state_estimation`，接收消息类型为`nav_msgs::Odometry`，并把接收到的的消息存储在curTime, vehicleX, vehicleY, vehicleZ中

#### goal_callback

goal_callback作为回调函数，接收`geometry_msgs::PoseStamped`类型的消息，并把其转化为endPoint，然后从全局变量vehicleX, vehicleY中得到startPoint，而vehicleX, vehicleY的值又是从topic `/state_estimation`订阅得到的，相当于两个订阅的topic `/state_estimation`与`/move_base_simple/goal`分别提供了startPoint与endPoint信息

结合topoMap，得到距离这两点最近的topoMap上的点，startNode, endNode，通过函数bfs得到一条从startNode到endNode可行的拓扑路径，加上startPoint, endPoint存储在waypoints上，将这些点转化为点云数据通过topic `topoPoint`发布，然后将这些点以线段形式通过topic `topoMap`发布

---

此为订阅回调函数，订阅topic `/move_base_simple/goal`, 消息类型`geometry_msgs::PoseStamped`

1.定义`pcl::PointXYZ`类型的point，将msg，即订阅得到的`geometry_msgs::PoseStamped`类型消息中的pose.position的x,y,z信息转入给point.x, y, z

2.给全局变量startPoint, endPoint赋值，如下

```c++
startPoint.x = vehicleX, startPoint.y = vehicleY;
endPoint.x = point.x, endPoint.y = point.y;
```

其中用到vehicleX，vehicleY为全局变量，endPoint即为订阅得到的信息

3.定义`routePoint *`类型变量startNode，遍历topoMap，之前在main里面引用过，将一个文件里所有拓扑点信息导入到了其中。计算topoMap中所有点与startPoint的距离的平方，记startNode=topoMap[i]，这里topoMap[i]为与startPoint距离最短的topoMap里的点

**注意**，这里设置了startMinScore=1000，故与startPoint距离最短的拓扑点距离的平方，应当要小于1000，否则startNode未赋值

4.类型于3，不过这次topoMap与endPoint进行比较，将结果导入endNode中，同样有endMinScore=1000，至少要满足该要求

这样，通过3,4，我们得到了startNode, endNode，分别对应距离startPoint, endPoint距离最短的拓扑点

5.定义`map<routePoint* , int>`类型变量flag, 将每个topoMap[i]作为键，其对应值为0，将startNode, endNode作为参数传入bfs，得到一条从startNode到endNode的拓扑路径，存储在path中

6.将waypoints清空，依次加入startPoint-->path--->endPoint，注意path是指从startNode到endNode的拓扑路径，并非是从startPoint到endPoint的路径

7.定义pcl::PointCloud\<pcl::PointXYZ>cloud类型数据，将waypoints数据转到cloud.points数据，定义`sensor_msgs::PointCloud2`类型数据output，将cloud中的数据转换到output中，然后通过topic `topoPoint`发布

8.定义`visualization_msgs::Marker`类型数据line_list_topoMap, 将waypoints中的点信息以连续线段的形式存储到line_list_topoMap中，然后通过topic `topoMap`发布

#### bfs

该函数依据topoMap，找到一条可能存在的从startNode到endNode的路径，存储在path中，其本质是routePoint * 的vector对象

在goal_callback中被调用

```c++
bool bfs(routePoint *startNode, routePoint *endNode, vector<routePoint *> &path, map<routePoint *, int> &flag)
```

1.定义`queue<routePoint *>`类型变量processNode，并添加startNode，设置flag[startNode] = 1, 设置bool类型find = false, 设置`routePoint *`类型变量endNodeSearch

2.进入循环，该循环从startNode开始，遍历它每个邻近点，邻近点的邻近点，如果能够找到一条从startNode通往endNode的拓扑路径，则find=true，否则find=false

3.如果find=true，即从startPoint到endPoint之间存在一条拓扑路径，那么将这一条按顺序路径存储到path中，**可能存在着多条路径，但这里只要了最先找到的那条**

#### topic

![image-20220318103559479](https://gitee.com/ccxc1001/image/raw/master/redmi_ubuntu_img/image-20220318103559479.png)

### rvizWayPoint

---

这部分为globalPlanner的拙劣模仿，依旧订阅`/state_estimation`与`/move_base_simple/goal`，但是`/state_estimation`没有起到任何作用，本部分唯一作用在于从topic `/move_base_simple/goal`接收目标点信息，然后不生成拓扑路径直接发布到topic  `way_point`, 所以它没有子目标点，只需要不断到达所发布的点

---

- 全局变量

curTime: double类型，默认值为0，在topic `state_estimation` 回调函数`poseHandler`中被引用赋值，设置为当前时间

vehicleX: float类型，默认值为0，在topic `state_estimation` 回调函数`poseHandler`中被引用赋值，为当前小车所在位置

vehicleY: float类型，默认值为0，在topic `state_estimation` 回调函数`poseHandler`中被引用赋值，为当前小车所在位置

vehicleZ: float类型，默认值为0，在topic `state_estimation` 回调函数`poseHandler`中被引用赋值，为当前小车所在位置

#### main

1.设置节点`waypointExample`，不过它会在`rvizWayPoint.launch`文件里被重命名为`rvizWaypoint`，获取参数，都写在`rvizWayPoint.launch`文件里

2.设置节点订阅topic `/state_estimation`，回调函数`poseHandler`，句柄subPose

3.设置节点将要发布到topic `/speed`，句柄pubSpeed

4.设置节点将要发布到topic `/navigation_boundary`，句柄pubBoundary

5.设置节点将要发布到topic `/way_point`，句柄pubWaypoint

6.设置节点订阅topic `/move_base_simple/goal`，回调函数goal_callback，句柄subWaypoint

7.全局变量sendBoundary为true，读取文件`boundary.ply`, 存储到变量boundary，将其转化为`geometry_msgs::PolygonStamped`形式

#### poseHandler

此为topic `state_estimation`的回调函数，它从订阅得到的信息里取出时间，位置分别存储到全局变量curTime, vehicleX, vehicleY, vehicleZ

#### goal_callback

此为topic `/move_base_simple/goal`的回调函数，从订阅信息抽取位置信息，转换为`geometry_msgs::PointStamped`格式，发布到topic `way_point`

#### topic

![image-20220318195108268](https://gitee.com/ccxc1001/image/raw/master/readme_window_image/image-20220318195108268.png)

### waypointExample

---

这部分所做的事情和`globalPlanner`很像，都是从topic `/state_estimation`获取小车当前位置信息，不同于`globalPlanner`从topic `/move_base_simple/goal`获取目标点信息生成拓扑路径，这里直接读取waypoints.ply文件获得拓扑路径信息

之后这里还有一个限制，到达子目标点之后，需要等一段waitTime时间，才能发布下一个子目标点，在循环，也发布信息到topic `way_point`, `/speed`, `/navigation_boundary`

---

- 全局变量

waypoints: `pcl::PointCloud<pcl::PointXYZ>::Pt`类型，存储从waypoints.ply文件读取的全部点信息

vehicleX: float类型，默认值为0，存储小车当前位置信息

vehicleY: float类型，默认值为0，存储小车当前位置信息

vehicleZ: float类型，默认值为0，存储小车当前位置信息

---

#### main

1.初始化节点，获取在launch文件中定义的参数

2.设置节点，订阅topic `/state_estimation`, 回调函数poseHandler，句柄subPose

3.设置节点准备发布，发布到topic `/way_point`，句柄pubWaypoint

4.设置节点准备发布，发布到topic `/speed`，句柄为pubSpeed

5.设置节点准备发布，发布到topic `/navigation_boundary`, 句柄为pubBoundary

6.运行函数`readWaypointFile`, 文件路径在waypoint_example.launch文件里，读取waypoints.ply文件，将其中所有点信息存储在`waypoints`中

7.如果全局变量`sendBoundary`设置为true, 则从boundary.ply文件读取数据，存储在全局变量`boundary`中，将boundary数据转换到boundaryMsgs中

8.做类似于`globalPlanner`中while循环内的事情，不过这里依据读取`waypoints`文件获得拓扑路径，在`globalPlanner`里依据topoMap得到拓扑路径，但之后的操作一致，到达子目标点，再发布下一个子目标点，这里还多了一个限制条件，只有当在到达子目标点之后，经过大于全局变量`waitTime`的时间，才发布下一个子目标点

9.以一定频率将子目标点位置，以及当前时间也就是curTime发布到topic `way_point`

10.如果全局变量sendSpeed设置为true，则发布速度全局变量speed到topic `/speed`

11.如果全局变量seendBoundary设置为true, 则发布边界信息到topic `/navigation_boundary`

#### poseHandler

此为topic `state_estimation`的回调函数，它从订阅得到的信息里取出时间，位置分别存储到全局变量curTime, vehicleX, vehicleY, vehicleZ

# velodyne-master

