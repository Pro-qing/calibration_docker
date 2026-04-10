# 雷达标定

## 1. 16线雷达标定流程
1. 确保车辆停放在三面墙之间，U型墙体中间，车辆正对前墙，尽量正对即可，确保车辆周围没有障碍物。

2. 使用卷尺测量16线雷达中心位置到地面的垂直距离，单位为m。

3. 使用卷尺测量后轮距，包含插肩即可，单位为m。

4. 使用卷尺分别测量左右后轮到前墙的距离，单位为m。
   
5. 使用卷尺分别测量左右后轮到两侧墙体的距离，单位为m；左轮到左墙，右轮到右墙。

6. 可以用卷尺测量雷达到base_link的大致数值放入laucnh中。

7. 打开lidar_self_calibration功能包，将对应数据填入launch文件中。 

8. 编译运行launch即可。

9. 最终结果会输出到lidar_self_calibration功能包下的param/lidar_calibration.yaml文件中 

## 2. mid雷达标定流程

1. 确保车辆停放在三面墙之间，U型墙体中间，车辆正对前墙，尽量正对即可，确保车辆周围没有障碍物。

2. 使用卷尺测量16线雷达中心位置到地面的垂直距离，单位为m。

3. 使用向日葵进行剩下操作

4. 开启bag包录制，rosbag record /points_16 /points_mid /tf  

5. 打开self_calibration_mid功能包，将对应数据填入launch文件中。

6. 运行launch文件，即可进行标定，最终结果会输出到self_calibration_mid功能包下的param/lidar_calibration.yaml文件中