cmake_minimum_required(VERSION 3.0.2)
project(lidar_self_calibration)

# 1. 查找依赖包 (集中放在一起)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_conversions
)
find_package(PCL 1.10 REQUIRED)
find_package(yaml-cpp REQUIRED)

message(STATUS "PCL_VERSION = ${PCL_VERSION}")

# 2. 引入头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2 tf2_ros pcl_conversions
)

# 3. 编译节点 (名字统一加 _node 后缀，与 launch 文件完全对应)
add_executable(${PROJECT_NAME}_node src/lidar_self_calibration.cpp src/main.cpp)

# 4. 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

#############
## Install ##
#############

# 安装头文件 (修正为匹配 *.hpp)
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

# 安装 launch 目录
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装可执行文件 (放到 ROS 环境能够识别的 bin 目录下)
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)