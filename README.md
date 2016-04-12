# orb_init
ORB_SLAM  automatic initalization。When this node started,xiaoqiang first will move arround and force ORB_SLAM get tracked.Then xiaoqiang continue to move two circles (2*2 meters^2 free space needed )in order to cause a  Loop Closing。Finally，we would get the continually output scaled visual odometry.

##input topic: 
	/system_monitor/report

	/ORB_SLAM/Camera

	/xqserial_server/Pose2D

##output topic: 
	/orb_scale/scaleStatus  (1hz)

	/ORB_SLAM/Odom          (visual odometry)

	/cmd_vel  (occasionally)

##output tf: 
	odom_combined-->/base_footprint

	base_link-->camera

##Quickstart：
rosrun orb_init orb_scale.py 

##Made with :heart: by BlueWhale Tech corp.



用于自动初始化ORB_SLAM，同时确定scale因子。初始化过程小车会自主移动两圈使ORB地图产生一次loop Closing，需要提供个一个2米*2米的空间。
将ORB_SLAM发出的topic(/ORB_SLAM/Camera)数据转换成odom_combined坐标系下的里程计topic(/ORB_SLAM/Odom)，这个就是视觉里程计，同时发布两个tf： odom_combined-->/base_footprint base_link-->camera。

简单使用方法：
rosrun orb_init orb_scale.py 

由蓝鲸科技精 :heart: 制作。
