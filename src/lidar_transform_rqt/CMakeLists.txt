cmake_minimum_required(VERSION 3.0.2)
project(lidar_transform_rqt)

add_compile_options(-std=c++14)

find_package(PCL 1.10 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  sensor_msgs
  # pcl_ros
  pcl_conversions
  laser_geometry
)

find_package(Qt5Widgets REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui_cpp sensor_msgs pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR} # 用于找到生成的 ui_*.h
)

# 1. 设置 UI 文件
set(UI_FILES resource/lidar_transform.ui)
qt5_wrap_ui(UI_HEADERS ${UI_FILES})

# 2. !!! 关键修复步骤 !!!
# 显式指定需要 MOC 的头文件
set(MOC_HEADERS
  include/lidar_transform_rqt/lidar_transform.h
)

# 手动运行 MOC
qt5_wrap_cpp(MOC_SRCS ${MOC_HEADERS})

# 3. 生成库
# 必须把 ${MOC_SRCS} 加入到 add_library 中！
add_library(${PROJECT_NAME}
  src/lidar_transform.cpp
  ${MOC_SRCS}
  ${UI_HEADERS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
  ${PCL_LIBRARIES}
)

# 确保生成的头文件在编译源码前就已经存在
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)