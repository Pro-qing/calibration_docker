#include "lidar_self_calibration/lidar_self_calibration.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <clocale>
#include <cmath>
#include <iomanip>

// 构造函数
LidarCalibration::LidarCalibration(ros::NodeHandle& nh) : nh_(nh)
{
    std::setlocale(LC_ALL, ""); 
    
    nh_.param<std::string>("lidar_frame", lidar_frame_, "velodyne");
    nh_.param<std::string>("base_frame", base_frame_, "base_link");
    nh_.param<std::string>("save_path", save_path_, "");

    // 1. 读取车轮物理测量参数
    float dist_l_front, dist_r_front;
    float dist_l_left, dist_r_right;
    float wheel_track;
    
    nh_.param<float>("dist_left_wheel_to_front", dist_l_front, 2.0f);  
    nh_.param<float>("dist_right_wheel_to_front", dist_r_front, 2.0f); 
    nh_.param<float>("dist_left_wheel_to_left_wall", dist_l_left, 2.0f);  
    nh_.param<float>("dist_right_wheel_to_right_wall", dist_r_right, 2.0f);
    nh_.param<float>("wheel_track", wheel_track, 1.6f); 

    // [核心推导] 计算中心距离及停车倾角
    actual_front_dist_ = (dist_l_front + dist_r_front) / 2.0f;
    
    float raw_sin = (dist_l_front - dist_r_front) / wheel_track;
    float sin_val = std::max(-1.0f, std::min(1.0f, raw_sin));

    parking_yaw_rad_ = std::asin(sin_val);

    float half_track_proj = (wheel_track / 2.0f) * std::cos(parking_yaw_rad_);
    actual_left_dist_  = dist_l_left + half_track_proj;
    actual_right_dist_ = dist_r_right + half_track_proj;

    // 2. 读取雷达粗略安装位置
    nh_.param<float>("guess_lidar_x", guess_lidar_x_, 0.0f);
    nh_.param<float>("guess_lidar_y", guess_lidar_y_, 0.0f);
    nh_.param<float>("guess_lidar_yaw_deg", guess_lidar_yaw_deg_, 0.0f);
    nh_.param<float>("manual_lidar_height", manual_lidar_height_, 1.0f); 

    // 初始化 ROS 通信
    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_16", 1, &LidarCalibration::pointCloudCallback, this);
    filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", 1);
    calibrated_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("calibrated_points", 1);
    wall_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("wall_points", 1);
    status_pub_ = nh_.advertise<nav_msgs::Odometry>("calibration_status", 1);
    
    calibration_done_ = false;
    transform_matrix_.setIdentity();
    total_frames_ = 0;
    
    ROS_INFO("================================================");
    ROS_INFO("          三面墙自动标定节点已启动 ");
    ROS_INFO("      基于输入计算的 Base_link 真实位姿:");
    ROS_INFO("        -> 到前墙距离 : %.3f m", actual_front_dist_);
    ROS_INFO("        -> 到左墙距离 : %.3f m (含倾斜投影 %.3fm)", actual_left_dist_, half_track_proj);
    ROS_INFO("        -> 到右墙距离 : %.3f m (含倾斜投影 %.3fm)", actual_right_dist_, half_track_proj);
    ROS_INFO("        -> 车辆停车偏角: %.2f deg", toDegrees(parking_yaw_rad_));
    ROS_INFO("      雷达外参粗略先验:");
    ROS_INFO("        -> (X=%.2f, Y=%.2f, Yaw=%.1f deg)", guess_lidar_x_, guess_lidar_y_, guess_lidar_yaw_deg_);
    ROS_INFO("================================================");
}

void LidarCalibration::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    total_frames_++;
    current_header_ = msg->header;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.5, 1.5);
    pass.filter(*z_filtered_cloud);

    sensor_msgs::PointCloud2 debug_msg;
    pcl::toROSMsg(*z_filtered_cloud, debug_msg);
    debug_msg.header = current_header_; 
    filtered_pub_.publish(debug_msg);
    
    if (detectThreeLines(z_filtered_cloud)) {
        publishWallPoints();
        calculateCalibrationStep(); 
        accumulateAndSmooth();      
        publishStatus();
        
        if (calibration_done_) {
            publishTF();
            publishCalibratedCloud(z_filtered_cloud); 
            if (total_frames_ % 50 == 0) printCalibrationResults();
        }
    } 
}

Eigen::Vector3f LidarCalibration::refineLineParametersPCA(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<int>& indices) 
{
    if (indices.size() < 5) return Eigen::Vector3f::Zero();
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, indices, centroid);
    Eigen::Matrix3f cov;
    pcl::computeCovarianceMatrix(*cloud, indices, centroid, cov);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov, Eigen::ComputeEigenvectors);
    
    Eigen::Vector3f normal = es.eigenvectors().col(0);
    normal.z() = 0.0f;
    normal.normalize();

    if (normal.dot(centroid.head<3>()) > 0) normal = -normal;
    return Eigen::Vector3f(normal.x(), normal.y(), std::abs(normal.dot(centroid.head<3>())));
}

bool LidarCalibration::detectThreeLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (cloud->empty()) return false;
    std::vector<DetectedWall> detected_walls = detectWallsWithRANSAC(cloud);
    
    bool has_left = false, has_right = false, has_front = false;
    for(const auto& w : detected_walls) {
        if(w.label == "left") has_left = true;
        if(w.label == "right") has_right = true;
        if(w.label == "front") has_front = true;
    }

    if (!has_left || !has_right || !has_front) return false;
    
    for (const auto& wall : detected_walls) {
        LaserLine line;
        line.label = wall.label;
        line.point_count = wall.point_count;
        line.raw_points = wall.points;
        line.avg_distance = wall.avg_distance; 
        line.normal = wall.pca_normal;         
        line.angle_center = std::atan2(wall.pca_normal.y(), wall.pca_normal.x());
        line.confidence = std::min(1.0f, line.point_count / 100.0f);
        
        if (wall.label == "left") left_line_ = line;
        else if (wall.label == "right") right_line_ = line;
        else if (wall.label == "front") front_line_ = line;
    }
    return true;
}

std::vector<DetectedWall> LidarCalibration::detectWallsWithRANSAC(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<DetectedWall> walls;
    
    // 结合雷达的粗略偏置推算预期的墙面距离
    float exp_f = actual_front_dist_ - guess_lidar_x_;
    float exp_l = actual_left_dist_  - guess_lidar_y_;
    float exp_r = actual_right_dist_ + guess_lidar_y_; 
    float g_yaw = guess_lidar_yaw_deg_;

    struct WallRegion { std::string label; float min_ang; float max_ang; float exp_dist; };
    std::vector<WallRegion> wall_regions = {
        {"front", -40.0f - g_yaw,  40.0f - g_yaw, exp_f},
        {"left",   50.0f - g_yaw, 130.0f - g_yaw, exp_l},
        {"right",-130.0f - g_yaw, -50.0f - g_yaw, exp_r}
    };
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.10); 
    seg.setMaxIterations(500);
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    for (const auto& region : wall_regions) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr region_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        float min_rad = toRadians(region.min_ang);
        float max_rad = toRadians(region.max_ang);
        
        for (const auto& point : cloud->points) {
            float r = std::sqrt(point.x*point.x + point.y*point.y);
            
            // 先验距离保护: ±1.0米宽容度
            float min_r = std::max(0.1f, region.exp_dist - 1.0f);
            float max_r = region.exp_dist + 1.0f;
            if(r < min_r || r > max_r) continue; 

            float angle = std::atan2(point.y, point.x);
            while(angle < min_rad) angle += 2*M_PI;
            while(angle > max_rad + 2*M_PI) angle -= 2*M_PI;

            if (angle >= min_rad && angle <= max_rad) {
                region_cloud->push_back(point);
            }
        }
        
        if (region_cloud->size() < 30) continue;
        
        seg.setInputCloud(region_cloud);
        seg.segment(*inliers, *coefficients);
        
        if (inliers->indices.size() < 20) continue;
        
        Eigen::Vector3f pca_result = refineLineParametersPCA(region_cloud, inliers->indices);
        
        DetectedWall wall;
        wall.label = region.label;
        wall.avg_distance = pca_result.z();
        wall.pca_normal = Eigen::Vector3f(pca_result.x(), pca_result.y(), 0.0f);
        wall.point_count = inliers->indices.size();
        
        for (int idx : inliers->indices) wall.points.push_back(region_cloud->points[idx]);
        walls.push_back(wall);
    }
    return walls;
}

void LidarCalibration::calculateCalibrationStep()
{
    if (left_line_.point_count == 0 || right_line_.point_count == 0 || front_line_.point_count == 0) return;

    float meas_yaw_front = front_line_.angle_center;
    float meas_yaw_left  = left_line_.angle_center;
    float meas_yaw_right = right_line_.angle_center;

    float yaw_est_front = normalizeAngleRad(M_PI - meas_yaw_front);
    float yaw_est_left  = normalizeAngleRad(-M_PI/2.0f - meas_yaw_left);
    float yaw_est_right = normalizeAngleRad(M_PI/2.0f - meas_yaw_right);

    float w_f = front_line_.confidence * 10.0f; 
    float w_l = left_line_.confidence;
    float w_r = right_line_.confidence;

    float sum_sin = w_f*sin(yaw_est_front) + w_l*sin(yaw_est_left) + w_r*sin(yaw_est_right);
    float sum_cos = w_f*cos(yaw_est_front) + w_l*cos(yaw_est_left) + w_r*cos(yaw_est_right);
    float lidar_yaw_to_room = std::atan2(sum_sin, sum_cos);

    float final_lidar_yaw = lidar_yaw_to_room - parking_yaw_rad_;

    float tx_room = actual_front_dist_ - front_line_.avg_distance;
    float ty_from_left  = actual_left_dist_ - left_line_.avg_distance;
    float ty_from_right = right_line_.avg_distance - actual_right_dist_;
    
    float ty_room = 0.0f;
    if (w_l + w_r > 1e-5) {
        ty_room = (ty_from_left * w_l + ty_from_right * w_r) / (w_l + w_r);
    }

    float cos_p = std::cos(parking_yaw_rad_);
    float sin_p = std::sin(parking_yaw_rad_);
    float tx_base = tx_room * cos_p + ty_room * sin_p;
    float ty_base = -tx_room * sin_p + ty_room * cos_p;

    transform_matrix_.setIdentity();
    Eigen::Matrix3f R = Eigen::AngleAxisf(final_lidar_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    transform_matrix_.block<3,3>(0,0) = R;
    transform_matrix_(0,3) = tx_base;
    transform_matrix_(1,3) = ty_base;
    transform_matrix_(2,3) = manual_lidar_height_;
}

void LidarCalibration::accumulateAndSmooth()
{
    float current_yaw = std::atan2(transform_matrix_(1,0), transform_matrix_(0,0));
    float ty_left_check  = actual_left_dist_ - left_line_.avg_distance;
    float ty_right_check = right_line_.avg_distance - actual_right_dist_;
    float right_error = std::abs(ty_left_check - ty_right_check);

    CalibrationResult res;
    res.transform = transform_matrix_;
    res.yaw = current_yaw;
    res.right_error = right_error;
    res.quality = left_line_.confidence + right_line_.confidence + front_line_.confidence;

    calibration_queue_.push_back(res);
    if (calibration_queue_.size() > 50) calibration_queue_.pop_front();
    
    computeRobustAverage();

    if (calibration_queue_.size() >= 50 && total_frames_ % 200 == 0) {
        saveResultsToYaml();
    }
}

void LidarCalibration::computeRobustAverage()
{
    if (calibration_queue_.size() < 10) return; 

    double sum_sin = 0, sum_cos = 0, sum_x = 0, sum_y = 0;
    int count = 0;

    for (const auto& res : calibration_queue_) {
        sum_sin += std::sin(res.yaw);
        sum_cos += std::cos(res.yaw);
        sum_x += res.transform(0, 3);
        sum_y += res.transform(1, 3);
        count++;
    }

    float final_yaw = std::atan2(sum_sin / count, sum_cos / count);
    
    transform_matrix_.setIdentity();
    transform_matrix_.block<3,3>(0,0) = Eigen::AngleAxisf(final_yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    transform_matrix_(0,3) = sum_x / count;
    transform_matrix_(1,3) = sum_y / count;
    transform_matrix_(2,3) = manual_lidar_height_;
    
    calibration_done_ = true;
}

void LidarCalibration::saveResultsToYaml()
{
    if (save_path_.empty()) return;
    try {
        YAML::Node config;
        config["lidar_calibration"]["x"] = transform_matrix_(0, 3);
        config["lidar_calibration"]["y"] = transform_matrix_(1, 3);
        config["lidar_calibration"]["z"] = transform_matrix_(2, 3);
        config["lidar_calibration"]["yaw"] = std::atan2(transform_matrix_(1, 0), transform_matrix_(0, 0));
        config["lidar_calibration"]["pitch"] = 0.0;
        config["lidar_calibration"]["roll"] = 0.0;
        config["lidar_calibration"]["frame_id"] = base_frame_;
        config["lidar_calibration"]["child_frame_id"] = lidar_frame_;

        std::ofstream fout(save_path_);
        fout << config;
        fout.close();
        ROS_INFO("标定参数已自动保存至: %s", save_path_.c_str());
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to write YAML: %s", e.what());
    }
}

void LidarCalibration::publishTF() {
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = current_header_.stamp; 
    ts.header.frame_id = base_frame_;
    ts.child_frame_id = lidar_frame_;
    
    ts.transform.translation.x = transform_matrix_(0, 3);
    ts.transform.translation.y = transform_matrix_(1, 3);
    ts.transform.translation.z = transform_matrix_(2, 3);
    
    Eigen::Quaternionf q(transform_matrix_.block<3, 3>(0, 0));
    ts.transform.rotation.x = q.x(); ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z(); ts.transform.rotation.w = q.w();
    
    tf_broadcaster_.sendTransform(ts);
}

void LidarCalibration::publishStatus() {
    nav_msgs::Odometry status;
    status.header.stamp = current_header_.stamp; 
    status.header.frame_id = base_frame_;
    status.child_frame_id = lidar_frame_;
    status.pose.pose.position.x = transform_matrix_(0, 3);
    status.pose.pose.position.y = transform_matrix_(1, 3);
    status.pose.pose.position.z = transform_matrix_(2, 3);
    Eigen::Quaternionf q(transform_matrix_.block<3, 3>(0, 0));
    status.pose.pose.orientation.x = q.x(); status.pose.pose.orientation.y = q.y();
    status.pose.pose.orientation.z = q.z(); status.pose.pose.orientation.w = q.w();
    status_pub_.publish(status);
}

void LidarCalibration::printCalibrationResults() {
    float x = transform_matrix_(0, 3), y = transform_matrix_(1, 3);
    float yaw = std::atan2(transform_matrix_(1, 0), transform_matrix_(0, 0));
    ROS_INFO("================ 标定输出 (稳定帧数: %lu) ================", calibration_queue_.size());
    ROS_INFO("最终标定结果: X=%.4f m, Y=%.4f m, Yaw=%.4f rad (%.2f deg)", x, y, yaw, toDegrees(yaw));
}

void LidarCalibration::publishWallPoints() {
    pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
    auto add_pts = [&](const LaserLine& line, int r, int g, int b) {
        for(auto& p : line.raw_points) {
            pcl::PointXYZRGB cp; cp.x=p.x; cp.y=p.y; cp.z=p.z; cp.r=r; cp.g=g; cp.b=b;
            colored_cloud.push_back(cp);
        }
    };
    add_pts(left_line_, 255, 0, 0);   
    add_pts(right_line_, 0, 255, 0);  
    add_pts(front_line_, 0, 0, 255);  
    
    if(!colored_cloud.empty()) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(colored_cloud, msg);
        msg.header = current_header_; 
        msg.header.frame_id = lidar_frame_;
        wall_points_pub_.publish(msg);
    }
}

void LidarCalibration::publishCalibratedCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed, transform_matrix_);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*transformed, msg);
    msg.header = current_header_; 
    msg.header.frame_id = base_frame_;
    calibrated_pub_.publish(msg);
}

float LidarCalibration::normalizeAngleRad(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

float LidarCalibration::toRadians(float degrees) { return degrees * M_PI / 180.0f; }
float LidarCalibration::toDegrees(float radians) { return radians * 180.0f / M_PI; }

void LidarCalibration::run() { ros::spin(); }