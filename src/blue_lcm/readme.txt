###先连接无线outblue

###对net.sh赋予权限
	sudo ./catkin_lcm/src/blue_lcm/src/net.sh

###再运行以下指令
	export LCM_DEFAULT_URL="udpm://224.0.0.1:7667?ttl=1"

###运行blue_lcm节点
	roslaunch blue_lcm blue_lcm.launch

###运行键盘控制指令
	roslaunch teleop_twist_keyboard_cpp teleop_twist_keyboard.launch
	(根据提示加减速)
