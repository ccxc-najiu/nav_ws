lgt created in 2021.11.16
分别按顺序在每个终端运行如下指令，空格之间的在一个终端打开
+


//录制数据
roslaunch velodyne_pointcloud VLP16_points.launch
rosbag record -o sia /velodyne_points

// 构建先验地图
roslaunch aloam_velodyne slam.launch
rosbag play --clock sia_2021-11-16-10-37-16.bag 

// 构建拓扑地图
roslaunch aloam_velodyne localization.launch
rosbag play --clock sia_2021-11-16-10-37-16.bag 
roslaunch data_generate topoGraph.launch


// 自主导航
roslaunch velodyne_pointcloud VLP16_points.launch

roslaunch aloam_velodyne localization.launch

roslaunch blue_lcm blue_lcm.launch


roslaunch vehicle_simulator systemGlobalPlanning.launch



