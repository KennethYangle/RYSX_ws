＃将两个launch文件放到/opt/ros/kinetic/share/mavros/launch 下面


＃启动px4 mavros topic  ACM0串口(以实际串口号为准)
roslaunch mavros px4.launch 
＃启动px4 mavros topic  ACM1串口
roslaunch mavros px4_ruying.launch

＃飞机端topic 对应　/mavros/下的各个topic
＃车辆端的topic 对应 /mavros_ruying/下的
/mavros_ruying/global_position/raw/fix   (gps 位置)
/mavros_ruying/global_position/raw/gps_vel　　(速度)
/mavros_ruying/local_position/pose　　(方向)
