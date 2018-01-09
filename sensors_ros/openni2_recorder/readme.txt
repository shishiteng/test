程序说明

phab2_msg_convert可以用pc录制tango图像和imu数据，具体过程：
1.在pc上建立一个5G的热点(一定要5G，否则无线传输的速度达不到实时传输)，开启roscore，tango连接热点
2.在tango(我用的lenovo phab2 pro)上开启ros streamer,配置master为pc，如果连接成功，ros streamer中的两个状态点都是绿色的，在pc端也能收到tango的message
3.数据录制
  打开一个终端,运行数据转换node
    rosrun tango phab2_msg_convert
  如果想要录制不同帧率的数据
    rosrun tango phab2_msg_convert [publish interval]
  如果interval设置为6,则每隔6帧publish一帧
  数据转换开启后，打开另一个终端，录制数据
    rosbag record /cam0/image_raw /imu0 [other topics...]



如果用phab2_msg_convert录制了数据集并跑了自己的算法，怎么对比自己算法和tango的区别呢？
这里提供了一个程序pose_convert，它可以把你的轨迹和tango的轨迹对齐(时间和空间)并实时显示。
具体用法：
1.建立一个node，发布tango的轨迹和自己的odometry
  publish ....
2.运行pose_convert
  rosrun tango pose_convert
这时候打开rviz，就可以看到和你的算法对齐的tango路径了。


文件说明

phab2_msg_convert.cpp
转换tango中ros streamer发布的消息
图像：从compressed image(30fps)转成raw image
imu：android/imu(200hz)转成 /imu0

pose_convert.cpp
对齐tango的轨迹和vins的odometry
输入：
    /vins_estimator/Odometry
    /tango/transform/start_of_service_T_device
输出：
    tango_path
    tango_pose

