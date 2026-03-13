#include "rqt_vehicle_calib_cpp/vehicle_calib.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QDoubleSpinBox>
#include <geometry_msgs/Point.h>

namespace rqt_vehicle_calib_cpp {

VehicleCalib::VehicleCalib()
  : rqt_gui_cpp::Plugin()
{
  setObjectName("VehicleCalibCpp");
}

void VehicleCalib::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  context.addWidget(widget_);

  // 初始化表格列宽
  ui_.table_points->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

  // 绑定信号槽
  connect(ui_.line_edit_topic, &QLineEdit::editingFinished, this, &VehicleCalib::onTopicChanged);
  connect(ui_.btn_add_point, &QPushButton::clicked, this, &VehicleCalib::onAddPoint);
  connect(ui_.btn_remove_point, &QPushButton::clicked, this, &VehicleCalib::onRemovePoint);
  connect(ui_.btn_publish, &QPushButton::clicked, this, &VehicleCalib::publishMarkers);
  connect(ui_.btn_generate_yaml, &QPushButton::clicked, this, &VehicleCalib::onGenerateYaml);
  
  // 表格内容改变自动发布
  connect(ui_.table_points, &QTableWidget::cellChanged, this, &VehicleCalib::onTableChanged);

  // 设置默认点位
  setupDefaultPoints();

  // 初始化发布者
  updatePublisher();
}

void VehicleCalib::shutdownPlugin()
{
  pub_marker_.shutdown();
}

void VehicleCalib::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // 可以保存当前的 Topic 名称等配置
  instance_settings.setValue("output_topic", ui_.line_edit_topic->text());
  instance_settings.setValue("frame_id", ui_.line_edit_frame->text());
}

void VehicleCalib::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  if (instance_settings.contains("output_topic")) {
    ui_.line_edit_topic->setText(instance_settings.value("output_topic").toString());
    updatePublisher();
  }
  if (instance_settings.contains("frame_id")) {
    ui_.line_edit_frame->setText(instance_settings.value("frame_id").toString());
  }
}

void VehicleCalib::updatePublisher()
{
  std::string topic = ui_.line_edit_topic->text().toStdString();
  if (topic.empty()) return;
  
  // 重新创建 Publisher
  pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>(topic, 1);
  publishMarkers();
}

void VehicleCalib::onTopicChanged()
{
  updatePublisher();
}

void VehicleCalib::onAddPoint()
{
  int row = ui_.table_points->rowCount();
  ui_.table_points->insertRow(row);
  
  // X Item
  QTableWidgetItem *item_x = new QTableWidgetItem("0.0");
  ui_.table_points->setItem(row, 0, item_x);
  
  // Y Item
  QTableWidgetItem *item_y = new QTableWidgetItem("0.0");
  ui_.table_points->setItem(row, 1, item_y);
}

void VehicleCalib::onRemovePoint()
{
  QList<QTableWidgetItem*> selected = ui_.table_points->selectedItems();
  if (selected.isEmpty()) return;
  
  // 简单处理：移除选中行（注意去重和索引顺序）
  int row = ui_.table_points->row(selected.first());
  ui_.table_points->removeRow(row);
  
  onTableChanged();
}

void VehicleCalib::setupDefaultPoints()
{
  // 暂时屏蔽信号，防止添加过程触发多次发布
  ui_.table_points->blockSignals(true);
  
  struct Pt { double x, y; };
  std::vector<Pt> defaults = {{1.0, -1.0}, {-1.0, -1.0}, {-1.0, 1.0}, {1.0, 1.0}};
  
  for (const auto& p : defaults) {
    int row = ui_.table_points->rowCount();
    ui_.table_points->insertRow(row);
    ui_.table_points->setItem(row, 0, new QTableWidgetItem(QString::number(p.x)));
    ui_.table_points->setItem(row, 1, new QTableWidgetItem(QString::number(p.y)));
  }
  
  ui_.table_points->blockSignals(false);
}

void VehicleCalib::onTableChanged()
{
  publishMarkers();
}

void VehicleCalib::publishMarkers()
{
  if (!pub_marker_) return;

  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  
  marker.header.frame_id = ui_.line_edit_frame->text().toStdString();
  marker.header.stamp = ros::Time::now();
  marker.ns = "manual_calib";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.03; // 线宽
  marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0; // 青色
  marker.pose.orientation.w = 1.0;

  // 读取表格数据
  std::vector<geometry_msgs::Point> points;
  for (int i = 0; i < ui_.table_points->rowCount(); ++i) {
    QTableWidgetItem* x_item = ui_.table_points->item(i, 0);
    QTableWidgetItem* y_item = ui_.table_points->item(i, 1);
    
    if (x_item && y_item) {
      geometry_msgs::Point p;
      p.x = x_item->text().toDouble();
      p.y = y_item->text().toDouble();
      p.z = -1.0; // 稍微放低
      points.push_back(p);
    }
  }

  // 闭合矩形
  if (!points.empty()) {
    points.push_back(points[0]);
  }
  
  marker.points = points;
  marker_array.markers.push_back(marker);
  
  pub_marker_.publish(marker_array);
}

void VehicleCalib::onGenerateYaml()
{
  QString yaml = "rect: [";
  QStringList parts;
  
  for (int i = 0; i < ui_.table_points->rowCount(); ++i) {
    QTableWidgetItem* x_item = ui_.table_points->item(i, 0);
    QTableWidgetItem* y_item = ui_.table_points->item(i, 1);
    
    if (x_item && y_item) {
      double x = x_item->text().toDouble();
      double y = y_item->text().toDouble();
      // 格式: {x: 1.0, y: -1.0}
      parts << QString("{x: %1, y: %2}").arg(x, 0, 'f', 3).arg(y, 0, 'f', 3);
    }
  }
  
  yaml += parts.join(", ");
  yaml += "]";
  
  ui_.text_output->setText(yaml);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(rqt_vehicle_calib_cpp::VehicleCalib, rqt_gui_cpp::Plugin)