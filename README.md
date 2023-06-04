# djkstra_-_path_record
1、在zxz_robot功能包中加入path_record功能包，依靠仿真环境输出的odom信息录制轨迹<br>
<br>
<br>
path_record功能包使用<br>
只需将path_record.cpp中的path_file路径和文件名称设置好，运行rosrun path_record path_record_node（在此之前需要启动仿真环境），即可录制轨迹，轨迹只可分段录制每次录制完要在轨迹前加入<br>
<br>
road 道路编号（例如road 1）<br>
point<br>
<br>
还需在轨迹后加入<br>
<br>
pre		     (前一段道路的编号)<br>
beh 2 3 4	（后一段道路的编号）<br>
<br>
<br>
<br>
<br>
2、加入by_djstl功能包<br>
<br>
<br>
设置好djstl_main.cpp中的roadMap_path，确定地图的地址和名称后运行<br>
roslaunch by_djstl 1.launch（改launch已经启动仿真环境）<br>
在启动的rviz中点击2D Nav Goal后点击目标点进行全局路径规划<br>
![image](https://github.com/bydsg/djkstra_-_path_record/blob/main/pic/2023-06-04%2017-21-36%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)



