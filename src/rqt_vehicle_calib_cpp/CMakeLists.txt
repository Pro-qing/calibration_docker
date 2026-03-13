cmake_minimum_required(VERSION 3.0.2)
project(rqt_vehicle_calib_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  visualization_msgs
  geometry_msgs
)

# 查找 Qt5 组件
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# 开启 MOC (Meta-Object Compiler)，用于处理 Q_OBJECT
set(CMAKE_AUTOMOC ON)
# 【修改点1】关闭 AUTOUIC，我们手动处理 ui 文件
set(CMAKE_AUTOUIC OFF) 
set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rqt_vehicle_calib_cpp
  CATKIN_DEPENDS roscpp rqt_gui_cpp visualization_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR} # 确保能找到生成的 ui_vehicle_calib.h
)

# 定义源代码
set(PLUGIN_SOURCES
  src/vehicle_calib.cpp
)

# 定义头文件
set(PLUGIN_HEADERS
  include/rqt_vehicle_calib_cpp/vehicle_calib.h
)

# 【修改点2】手动处理 UI 文件
# 这行命令会根据 resource/vehicle_calib.ui 生成 ui_vehicle_calib.h
qt5_wrap_ui(UI_HEADERS resource/vehicle_calib.ui)

# 生成库
add_library(${PROJECT_NAME}
  ${PLUGIN_SOURCES}
  ${PLUGIN_HEADERS}
  ${UI_HEADERS} # 【修改点3】将生成的 UI 头文件加入编译目标
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

# 安装规则
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