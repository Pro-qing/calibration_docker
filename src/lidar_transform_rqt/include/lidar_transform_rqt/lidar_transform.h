#ifndef LIDAR_TRANSFORM_RQT_LIDAR_TRANSFORM_H
#define LIDAR_TRANSFORM_RQT_LIDAR_TRANSFORM_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_lidar_transform.h> // 对应 my_plugin.ui
#include <QWidget>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <string>
#include <map>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_transform_rqt
{

class LidarTransform : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  LidarTransform();
  
  virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;
  virtual void shutdownPlugin() override;
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

protected:
  Ui::LidarTransformWidget ui_;
  QWidget* widget_;

  ros::NodeHandle nh_;
  
  ros::Subscriber sub_points_; 
  ros::Subscriber sub_scan_;   

  // 统一输出为 PointCloud2，移除 pub_scan_
  ros::Publisher pub_points_; 

  laser_geometry::LaserProjection projector_; 

  double x_, y_, z_, roll_, pitch_, yaw_;
  
  std::string input_topic_;
  std::string output_topic_;
  std::string target_frame_id_;

  std::map<std::string, std::string> topic_types_;
  std::string current_type_; 

  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void scanCallback(const sensor_msgs::LaserScanConstPtr& msg);

protected slots:
  void updateTransformParams();
  void refreshTopics();
  void applySettings();
};

} // namespace lidar_transform_rqt

#endif