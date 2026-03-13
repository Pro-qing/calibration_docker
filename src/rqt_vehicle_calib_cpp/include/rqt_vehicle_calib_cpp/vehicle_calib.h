#ifndef RQT_VEHICLE_CALIB_CPP_VEHICLE_CALIB_H
#define RQT_VEHICLE_CALIB_CPP_VEHICLE_CALIB_H

#include <rqt_gui_cpp/plugin.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <QWidget>
#include <ui_vehicle_calib.h> // 自动生成的 UI 头文件

namespace rqt_vehicle_calib_cpp {

class VehicleCalib : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  VehicleCalib();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;
  virtual void shutdownPlugin() override;
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

protected slots:
  // UI 回调函数
  void onTopicChanged();
  void onAddPoint();
  void onRemovePoint();
  void onGenerateYaml();
  void onTableChanged(); // 当表格内容改变时触发
  void publishMarkers(); // 发布逻辑

private:
  Ui::VehicleCalibWidget ui_;
  QWidget* widget_;
  
  ros::NodeHandle nh_;
  ros::Publisher pub_marker_;
  
  void setupDefaultPoints();
  void updatePublisher();
};

} // namespace

#endif // RQT_VEHICLE_CALIB_CPP_VEHICLE_CALIB_H