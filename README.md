
导远组合导航570d的ros下解析程序

#### 发布的Topics

* **`imu`** ([sensor_msgs::Imu])

	包括六轴陀螺仪和加速度计，以及从三轴绝对方向角计算而来的 orientation.

* **`Chassis_RTK`** ([my_msgs::ChassisMsg])

	包括其他信息例如经纬度等，显然这是我自定义的格式，如果需要通用格式例如sensor_msgs::NavSatFix ,请自行填写

#### 依赖和安装
* **`依赖`**

	本项目依赖与ros的serial包，用于接受串口信息，安装方式如下：
	sudo apt-get install ros-kinetic-serial 

* **`安装`**
	mkdir src
	cd src
	git clone https://github.com/hilbertletanger/imu_decode.git
	cd ..
	catkin_make

* **`运行`**
	source devel/setup.bash
	roslaunch imu_serial_node demo.launch 

#### 参数

* **`port`** (string, default: "/dev/ttyACM0")

	串口名.


* **`imu_frame_id`** (string, default: "imu_base")

	Sets the name of the base frame for imu messages.

