# djkstra_-_path_record
在zxz_robot功能包中加入path_record功能包，依靠仿真环境输出的odom信息录制轨迹<br>
<br>
<br>
path_record功能包使用<br>
只需将path_record.cpp中的path_file路径和文件名称设置好，运行rosrun path_record path_record_node（在此之前需要启动仿真环境），即可录制轨迹，轨迹只可分段录制<br>
每次录制完要在轨迹前加入<br>
road 道路编号（例如road 1）<br>
point<br>
还需在轨迹后加入<br>
pre		 (前一段道路的编号)<br>
beh 2 3 4	（后一段道路的编号）<br>
<br>
<br>
加入by_djstl功能包<br>
设置好djstl_main.cpp中的roadMap_path，确定地图的地址和名称后<br>
运行roslaunch by_djstl 1.launch（改launch已经启动仿真环境）<br>
在启动的rviz中点击2D Nav Goal后点击目标点进行全局路径规划


