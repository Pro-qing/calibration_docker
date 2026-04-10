#include "lidar_transform_rqt/lidar_transform.h"
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <ros/master.h>

namespace lidar_transform_rqt
{

LidarTransform::LidarTransform()
  : rqt_gui_cpp::Plugin()
  , widget_(nullptr)
  , x_(0.0), y_(0.0), z_(0.0), roll_(0.0), pitch_(0.0), yaw_(0.0)
{
  setObjectName("LidarTransform");
}

void LidarTransform::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  context.addWidget(widget_);

  nh_ = getMTNodeHandle();

  connect(ui_.sb_x, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LidarTransform::updateTransformParams);
  connect(ui_.sb_y, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LidarTransform::updateTransformParams);
  connect(ui_.sb_z, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LidarTransform::updateTransformParams);
  connect(ui_.sb_roll, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LidarTransform::updateTransformParams);
  connect(ui_.sb_pitch, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LidarTransform::updateTransformParams);
  connect(ui_.sb_yaw, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &LidarTransform::updateTransformParams);

  connect(ui_.btn_refresh, &QPushButton::clicked, this, &LidarTransform::refreshTopics);
  connect(ui_.btn_apply, &QPushButton::clicked, this, &LidarTransform::applySettings);

  refreshTopics();
  if (ui_.cb_input_topic->count() > 0) {
      ui_.cb_input_topic->setCurrentIndex(0);
      applySettings();
  }
}

void LidarTransform::refreshTopics()
{
  ui_.cb_input_topic->clear();
  topic_types_.clear();
  
  ros::master::V_TopicInfo master_topics;
  if (ros::master::getTopics(master_topics)) {
    for (const auto& topic : master_topics) {
      if (topic.datatype == "sensor_msgs/PointCloud2" || 
          topic.datatype == "sensor_msgs/LaserScan") {
        ui_.cb_input_topic->addItem(QString::fromStdString(topic.name) + " [" + QString::fromStdString(topic.datatype) + "]");
        topic_types_[topic.name] = topic.datatype;
      }
    }
  }
}

void LidarTransform::applySettings()
{
  QString current_text = ui_.cb_input_topic->currentText();
  if (current_text.isEmpty()) return;

  std::string full_str = current_text.toStdString();
  std::string topic_name = full_str.substr(0, full_str.find(" ["));
  
  std::string new_out_topic = ui_.le_output_topic->text().toStdString();
  target_frame_id_ = ui_.le_frame_id->text().toStdString();

  if (topic_name.empty() || new_out_topic.empty()) return;

  std::string type = topic_types_[topic_name];
  if (type.empty()) { 
       ros::master::V_TopicInfo master_topics;
       if (ros::master::getTopics(master_topics)) {
         for(auto& t : master_topics) if(t.name == topic_name) type = t.datatype;
       }
  }
  current_type_ = type;

  // 1. 发布者设置：无论输入是什么，我们统一发布 PointCloud2
  if (new_out_topic != output_topic_ || !pub_points_) {
    output_topic_ = new_out_topic;
    pub_points_.shutdown();

    ROS_INFO("Creating PointCloud2 publisher for %s", output_topic_.c_str());
    pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
  }

  // 2. 订阅者设置：根据输入类型决定订阅回调
  if (topic_name != input_topic_) {
    input_topic_ = topic_name;
    sub_points_.shutdown();
    sub_scan_.shutdown();

    ROS_INFO_STREAM("Subscribing to " << input_topic_ << " (" << current_type_ << ")");

    if (current_type_ == "sensor_msgs/LaserScan") {
        sub_scan_ = nh_.subscribe(input_topic_, 1, &LidarTransform::scanCallback, this);
    } else {
        sub_points_ = nh_.subscribe(input_topic_, 1, &LidarTransform::pointCloudCallback, this);
    }
  }
}

void LidarTransform::updateTransformParams()
{
  x_ = ui_.sb_x->value();
  y_ = ui_.sb_y->value();
  z_ = ui_.sb_z->value();
  roll_ = ui_.sb_roll->value();
  pitch_ = ui_.sb_pitch->value();
  yaw_ = ui_.sb_yaw->value();
}

void LidarTransform::scanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    if (pub_points_.getNumSubscribers() == 0) return;

    // 1. 将 LaserScan 转换为 PointCloud2
    sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
    try {
        projector_.projectLaser(*msg, *cloud_msg);
    } catch (std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Failed to convert LaserScan to PointCloud2: %s", e.what());
        return;
    }

    // 2. 直接复用 pointCloudCallback 来处理 PCL 转换、TF变换及发布
    pointCloudCallback(cloud_msg);
}

void LidarTransform::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if (pub_points_.getNumSubscribers() == 0) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    try {
      pcl::fromROSMsg(*msg, *cloud_in);
    } catch (...) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
      try {
        pcl::fromROSMsg(*msg, *cloud_xyz);
        cloud_in->points.resize(cloud_xyz->points.size());
        cloud_in->width = cloud_xyz->width;
        cloud_in->height = cloud_xyz->height;
        cloud_in->is_dense = cloud_xyz->is_dense;
        for (size_t i = 0; i < cloud_xyz->points.size(); ++i) {
            cloud_in->points[i].x = cloud_xyz->points[i].x;
            cloud_in->points[i].y = cloud_xyz->points[i].y;
            cloud_in->points[i].z = cloud_xyz->points[i].z;
            cloud_in->points[i].intensity = 0;
        }
      } catch (std::exception& e) {
        ROS_ERROR_THROTTLE(1.0, "Failed to convert pointcloud: %s", e.what());
        return;
      }
    }

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << x_, y_, z_;
    transform.rotate(Eigen::AngleAxisf(roll_, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(pitch_, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(yaw_, Eigen::Vector3f::UnitZ()));

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*cloud_in, *cloud_out, transform);

    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_out, output_msg);
    output_msg.header.stamp = msg->header.stamp;
    if (!target_frame_id_.empty()) {
        output_msg.header.frame_id = target_frame_id_;
    } else {
        output_msg.header.frame_id = msg->header.frame_id;
    }

    pub_points_.publish(output_msg);
}

void LidarTransform::shutdownPlugin()
{
  sub_points_.shutdown();
  sub_scan_.shutdown();
  pub_points_.shutdown();
  // 已移除 pub_scan_
}

void LidarTransform::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  instance_settings.setValue("x", x_);
  instance_settings.setValue("y", y_);
  instance_settings.setValue("z", z_);
  instance_settings.setValue("roll", roll_);
  instance_settings.setValue("pitch", pitch_);
  instance_settings.setValue("yaw", yaw_);
  instance_settings.setValue("input_topic_full", ui_.cb_input_topic->currentText());
  instance_settings.setValue("output_topic", QString::fromStdString(output_topic_));
  instance_settings.setValue("frame_id", QString::fromStdString(target_frame_id_));
}

void LidarTransform::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  if(instance_settings.contains("x")) ui_.sb_x->setValue(instance_settings.value("x").toDouble());
  if(instance_settings.contains("y")) ui_.sb_y->setValue(instance_settings.value("y").toDouble());
  if(instance_settings.contains("z")) ui_.sb_z->setValue(instance_settings.value("z").toDouble());
  if(instance_settings.contains("roll")) ui_.sb_roll->setValue(instance_settings.value("roll").toDouble());
  if(instance_settings.contains("pitch")) ui_.sb_pitch->setValue(instance_settings.value("pitch").toDouble());
  if(instance_settings.contains("yaw")) ui_.sb_yaw->setValue(instance_settings.value("yaw").toDouble());
  
  if (instance_settings.contains("input_topic_full")) {
      QString saved_topic = instance_settings.value("input_topic_full").toString();
      if (ui_.cb_input_topic->findText(saved_topic) == -1) {
          ui_.cb_input_topic->addItem(saved_topic);
          std::string s = saved_topic.toStdString();
          std::string name = s.substr(0, s.find(" ["));
          if (s.find("LaserScan") != std::string::npos) topic_types_[name] = "sensor_msgs/LaserScan";
          else topic_types_[name] = "sensor_msgs/PointCloud2";
      }
      ui_.cb_input_topic->setCurrentText(saved_topic);
  }
  
  if (instance_settings.contains("output_topic")) {
      ui_.le_output_topic->setText(instance_settings.value("output_topic").toString());
  }
  if (instance_settings.contains("frame_id")) {
      ui_.le_frame_id->setText(instance_settings.value("frame_id").toString());
  }

  updateTransformParams();
  applySettings();
}

} // namespace lidar_transform_rqt

PLUGINLIB_EXPORT_CLASS(lidar_transform_rqt::LidarTransform, rqt_gui_cpp::Plugin)