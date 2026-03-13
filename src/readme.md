参数：
v4版本参数：
        <!-- ========================================== -->
        <!-- 场景配置: 车间墙面距离 (单位: 米) -->
        <!-- ========================================== -->
        <!-- 车辆中心(或BaseLink)到各墙面的实际距离，用于验证多线雷达自身的数据是否合理 -->
        <!-- 车间墙面的实际距离（必须根据实际车间测量设置） -->
        <param name="actual_left_distance" type="double" value="0.738" />
        <param name="actual_right_distance" type="double" value="0.658" />
        <param name="actual_front_distance" type="double" value="2.22" />
        <!-- <param name="actual_left_distance" type="double" value="2.103" />
        <param name="actual_right_distance" type="double" value="2.503" />
        <param name="actual_front_distance" type="double" value="9.19" /> -->

        <!-- ========================================== -->
        <!-- 传感器安装高度 (单位: 米) -->
        <!-- ========================================== -->
        <!-- 手动测量的多线雷达高度（必须根据实际车间测量设置）-->
        <param name="manual_lidar_height" type="double" value="1.95" />

        <!-- 手动测量的左单线雷达高度（必须根据实际车间测量设置）-->
        <param name="manual_left_laser_height" type="double" value="0.1" />
        
        <!-- 手动测量的右单线雷达高度（必须根据实际车间测量设置）-->
        <param name="manual_right_laser_height" type="double" value="0.1" /> 


V3版本参数：
        <param name="actual_left_distance" type="double" value="2.103" />
        <param name="actual_right_distance" type="double" value="2.503" />
        <param name="actual_front_distance" type="double" value="9.19" />
                
        <!-- 手动测量的雷达高度（必须根据实际车间测量设置）-->
        <param name="manual_lidar_height" type="double" value="1.95" />

v0版本参数：
        <!-- 车间墙面的实际距离（必须根据实际车间测量设置） -->
        <param name="actual_left_distance" type="double" value="0.738" />
        <param name="actual_right_distance" type="double" value="0.658" />
        <param name="actual_front_distance" type="double" value="2.22" />
                
        <!-- 手动测量的雷达高度（必须根据实际车间测量设置）-->
        <param name="manual_lidar_height" type="double" value="1.95" />


1.lidar_self_calibration功能包，然后启动命令是：roslaunch lidar_self_calibration lidar_self_calibration.launch
2.self_calibration_v2功能包，然后启动命令是：roslaunch self_calibration_v2 self_calibration_v4.launch
3.self_calibration_v4功能包，然后启动命令是：roslaunch self_calibration_v4 self_calibration_v4.launch

需要传递的参数有：
        <!-- 车间墙面的实际距离（必须根据实际车间测量设置） -->
        <param name="actual_left_distance" type="double" value="0.738" />
        <param name="actual_right_distance" type="double" value="0.658" />
        <param name="actual_front_distance" type="double" value="2.22" />

        <!-- ========================================== -->
        <!-- 传感器安装高度 (单位: 米) -->
        <!-- ========================================== -->
        <!-- 手动测量的多线雷达高度（必须根据实际车间测量设置）-->
        <param name="manual_lidar_height" type="double" value="1.95" />

        <!-- 手动测量的左单线雷达高度（必须根据实际车间测量设置）-->
        <param name="manual_left_laser_height" type="double" value="0.1" />
        
        <!-- 手动测量的右单线雷达高度（必须根据实际车间测量设置）-->
        <param name="manual_right_laser_height" type="double" value="0.1" /> 















# 车辆大小矩形各个点的位置(按顺序连接)
rect:
  - {x: 0.35, y: -0.20}
  - {x: 0.35, y: -0.40}
  - {x: -0.10, y: -0.40}
  - {x: -0.10, y: -0.80}
  - {x: -1.66, y: -0.80}
  - {x: -1.66, y: 0.75}
  - {x: -0.10, y: 0.75}
  - {x: -0.10, y: 0.45}
  - {x: 0.35, y: 0.45}
  - {x: 0.35, y: 0.20}


  - {x: 0.38, y: -0.23}
  - {x: 0.38, y: -0.43}
  - {x: -0.13, y: -0.43}
  - {x: -0.13, y: -0.83}
  - {x: -1.69, y: -0.83}
  - {x: -1.69, y: 0.78}
  - {x: -0.13, y: 0.78}
  - {x: -0.13, y: 0.48}
  - {x: 0.38, y: 0.48}
  - {x: 0.38, y: 0.23}






exigencyrect 停止矩形
slowrect 速度减慢矩形
reverse_exigencyrect 反向停止矩形
reverse_slowrect 反向慢速矩形 


<!-- 原始数据
# 车辆大小矩形各个点的位置(按顺序连接)
exigencyrect:
  - {x: 0.90, y: -0.20}
  - {x: 0.80, y: -0.52}
  - {x: -0.15, y: -0.52}
  - {x: -0.15, y: -0.70}
  - {x: -1.66, y: -0.70}
  - {x: -1.66, y: 0.65}
  - {x: -0.15, y: 0.65}
  - {x: -0.15, y: 0.56}
  - {x: 0.80, y: 0.56}
  - {x: 0.90, y: 0.20}

slowrect:
  - {x: 1.6, y: 0.9}
  - {x: 1.6, y: -0.9}
  - {x: -0.0, y: -1.10}
  - {x: -0.40, y: -1.12}
  - {x: -1.8, y: -1.12}
  - {x: -1.8, y: 1.12}
  - {x: -0.40, y: 1.12}
  - {x: -0.0, y: 1.10}


reverse_exigencyrect:
  - {x: 0.90, y: -0.20}
  - {x: 0.80, y: -0.60}
  - {x: -0.15, y: -0.60}
  - {x: -0.15, y: -0.75}
  - {x: -0.15, y: 0.67}
  - {x: -0.15, y: 0.56}
  - {x: 0.80, y: 0.56}
  - {x: 0.90, y: 0.20}

reverse_slowrect:
  - {x: 1.40, y: 0.80}
  - {x: 1.40, y: -0.80}
  - {x: -0.0, y: -1.06}
  - {x: -0.40, y: -1.08}
  - {x: -0.50, y: -1.08}
  - {x: -0.50, y: 1.08}
  - {x: -0.40, y: 1.08}
  - {x: -0.0, y: 1.06}

   -->





roam@user-Default-string:~/work/autoware.ai$ rostopic echo /lqr_targetwayp -n 1
gid: 0
lid: 0
pose: 
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs:         0
    frame_id: ''
  pose: 
    position: 
      x: -75.83789265468116
      y: 119.45661914104777
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.997419785886833
      w: 0.07178976752618793
twist: 
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs:         0
    frame_id: ''
  twist: 
    linear: 
      x: 0.6
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
dtlane: 
  dist: 0.0
  dir: 0.0
  apara: 0.0
  r: 0.0
  slope: 0.0
  cant: 0.0
  lw: 0.0
  rw: 0.0
change_flag: 0
wpstate: 
  aid: 0
  lanechange_state: 0
  steering_state: 0
  accel_state: 0
  stop_state: 0
  event_state: 0
wpsattr: 
  name: "233"
  floor: 1
  types: []
  routeBehavior: 4
lane_id: 0
left_lane_id: 0
right_lane_id: 0
stop_line_id: 0
cost: 0.0
time_cost: 0.0
direction: 0
---


getq@getq:~/workspace$ rostopic pub /lqr_targetwayp autoware_msgs/Waypoint "gid: 0id: 0
lid: 0er:
pose:der:0
  header:0 {secs: 0, nsecs: 0}
    seq: 0 {secs: 0, nsecs: 0}
    stamp: {secs: 0, nsecs: 0}
    frame_id: '': 0.0, y: 0.0, z: 0.0}
  pose:ition: {x: 0.0, y: 0.0, z: 0.0}.0, w: 0.0}
    position: {x: 0.0, y: 0.0, z: 0.0}.0, w: 0.0}
    orientation: {x^C0.0, y: 0.0, z: 0.0, w: 0.0}
twist:er:0
  header:0 {secs: 0, nsecs: 0}
    seq: 0 {secs: 0, nsecs: 0}
    stamp: {secs: 0, nsecs: 0}tate: 0, steering_state: 0, accel_state: 0, stop_st
    frame_id: ''
  twist:ar: {x: 0.0, y: 0.0, z: 0.0}
    linear: {x: 0.0, y: 0.0, z: 0.0}}
    angular: {x: 0.0, y: 0.0, z: 0.0}0.0, r: 0.0, slope: 0.0, cant: 0.0, lw: 0.0,
dtlane: {dist: 0.0, dir: 0.0, apara: 0.0, r: 0.0, slope: 0.0, cant: 0.0, lw: 0.0,
  rw: 0.0}
  rw: 0.0}g: 0
change_flag: 0 0, lanechange_state: 0, steering_state: 0, accel_state: 0, stop_st
wpstate: {aid: 0, lanechange_state: 0, steering_state: 0, accel_state: 0, stop_st
ate: 0,_state: 0}
  event_state: 0}
lane_id: 0id: 0
left_lane_id: 00
right_lane_id: 0
stop_line_id: 0
cost: 0.0: 0.0
time_cost: 0.0
direction: 0" 
getq@getq:~/workspace$ rostopic pub /lqr_
/lqr_dire        /lqr_targetwayp  
getq@getq:~/workspace$ rostopic pub /lqr_
/lqr_dire        /lqr_targetwayp  
getq@getq:~/workspace$ rostopic pub /lqr_targetwayp autoware_msgs/Waypoint "gid: 
4
lid: 0
pose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
twist:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  twist:
    linear: {x: 0.0, y: 0.0, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.0}
dtlane: {dist: 0.0, dir: 0.0, apara: 0.0, r: 0.0, slope: 0.0, cant: 0.0, lw: 0.0,
 
  rw: 0.0}
change_flag: 0
wpstate: {aid: 0, lanechange_state: 0, steering_state: 0, accel_state: 0, stop_st
ate: 0,
  event_state: 0}
lane_id: 0
left_lane_id: 0
right_lane_id: 0
stop_line_id: 0
cost: 0.0
time_cost: 0.0
direction: 0" 