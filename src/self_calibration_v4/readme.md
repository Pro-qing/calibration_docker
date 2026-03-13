#include "self_calibration_v4/self_calibration_v4.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/common/io.h> 
#include <clocale>
#include <cmath>
#include <iomanip>
#include <algorithm> 

// 构造函数
LidarCalibration::LidarCalibration(ros::NodeHandle& nh) : nh_(nh)
{
    std::setlocale(LC_ALL, "");

    nh_.param<std::string>("lidar_frame", lidar_frame_, "velodyne");
    nh_.param<std::string>("base_frame", base_frame_, "base_link");

    nh_.param<float>("actual_left_distance", actual_left_dist_, 0.0f);
    nh_.param<float>("actual_right_distance", actual_right_dist_, 0.0f);
    nh_.param<float>("actual_front_distance", actual_front_dist_, 0.0f);
    nh_.param<float>("manual_lidar_height", manual_lidar_height_, 1.0f);

    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_16", 1, &LidarCalibration::pointCloudCallback, this);
    scan_left_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_left", 1, &LidarCalibration::scanLeftCallback, this);
    scan_right_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_right", 1, &LidarCalibration::scanRightCallback, this);

    filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", 1);
    wall_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("wall_points", 1);
    
    scan_left_viz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_scan_left", 1);
    scan_right_viz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_scan_right", 1);

    T_base_to_velo_.setIdentity();
    velo_ready_ = false;

    left_calib_queue_.clear();
    right_calib_queue_.clear();

    ROS_INFO("================================================");
    ROS_INFO("   Lidar Calibration V4 (Strict Rules)");
    ROS_INFO("   Left Radar: Longest=Left Wall, 2nd=Front");
    ROS_INFO("   Right Radar: Longest=Right Wall, 2nd=Front");
    ROS_INFO("================================================");
}

// --------------------------------------------------------------------------------
// 1. Velodyne 基准计算
// --------------------------------------------------------------------------------
void LidarCalibration::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    if (cloud->empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr z_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-1.0, 1.0); 
    pass.filter(*z_filtered);

    filtered_pub_.publish(msg); 

    if (detectThreePlanes(z_filtered, velo_left_, velo_right_, velo_front_)) {
        calculateVeloToBase();
        velo_ready_ = true;

        pcl::PointCloud<pcl::PointXYZRGB> colored;
        auto add = [&](const LaserLine& l, int r, int g, int b) {
            for(auto p : l.raw_points) {
                pcl::PointXYZRGB cp; cp.x=p.x; cp.y=p.y; cp.z=p.z; cp.r=r; cp.g=g; cp.b=b; colored.push_back(cp);
            }
        };
        // Velodyne Wall Colors: Left=Red, Right=Green, Front=Blue
        add(velo_left_, 255,0,0); 
        add(velo_right_, 0,255,0); 
        add(velo_front_, 0,0,255);
        sensor_msgs::PointCloud2 out; pcl::toROSMsg(colored, out);
        out.header = msg->header; wall_points_pub_.publish(out);
    }
}

bool LidarCalibration::detectThreePlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                         LaserLine& left, LaserLine& right, LaserLine& front)
{
    if (cloud->empty()) return false;

    struct Region { std::string label; float min_a; float max_a; };
    std::vector<Region> regions = { {"front", -45, 45}, {"left", 45, 135}, {"right", -135, -45} };

    bool has_l=false, has_r=false, has_f=false;

    for (const auto& reg : regions) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr roi(new pcl::PointCloud<pcl::PointXYZ>);
        float min_rad = toRadians(reg.min_a);
        float max_rad = toRadians(reg.max_a);

        for (const auto& p : cloud->points) {
            float r = std::sqrt(p.x*p.x + p.y*p.y);
            if(r < 0.5 || r > 10.0) continue;
            float ang = std::atan2(p.y, p.x);
            if (ang >= min_rad && ang <= max_rad) roi->push_back(p);
        }

        if (roi->size() < 30) continue;

        pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(roi);
        seg.segment(*inliers, *coeff);

        if (inliers->indices.size() < 30) continue;

        Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
        Eigen::Vector4f c; pcl::compute3DCentroid(*roi, *inliers, c);
        
        // 强制法向量指向原点
        if (n.dot(c.head<3>()) > 0) n = -n;

        LaserLine l;
        l.label = reg.label;
        l.avg_distance = std::abs(coeff->values[3]);
        l.normal = n;
        l.confidence = inliers->indices.size();
        for(int idx : inliers->indices) l.raw_points.push_back(roi->points[idx]);

        if (reg.label == "left") { left = l; has_l = true; }
        else if (reg.label == "right") { right = l; has_r = true; }
        else if (reg.label == "front") { front = l; has_f = true; }
    }

    return has_l && has_r && has_f;
}

void LidarCalibration::calculateVeloToBase()
{
    Eigen::Vector3f x_axis = -velo_front_.normal;
    float conf_sum = velo_left_.confidence + velo_right_.confidence;
    Eigen::Vector3f y_axis;
    if (conf_sum > 0) {
        y_axis = (-velo_left_.normal * velo_left_.confidence + velo_right_.normal * velo_right_.confidence) / conf_sum;
    } else {
        y_axis = -velo_left_.normal;
    }
    y_axis = (y_axis - (y_axis.dot(x_axis)) * x_axis).normalized();
    Eigen::Vector3f z_axis = x_axis.cross(y_axis).normalized();

    Eigen::Matrix3f R;
    R.col(0) = x_axis; R.col(1) = y_axis; R.col(2) = z_axis;
    
    T_base_to_velo_.setIdentity();
    T_base_to_velo_.block<3,3>(0,0) = R.transpose();
}


// --------------------------------------------------------------------------------
// 2. 单线雷达标定逻辑
// --------------------------------------------------------------------------------

void LidarCalibration::scanLeftCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!velo_ready_) return;

    // 1. 转为点云
    sensor_msgs::PointCloud2 cloud_msg;
    projector_.projectLaser(*msg, cloud_msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *cloud);

    // ==========================================================
    // [新增] 倒装修正：绕 X 轴旋转 180 度 (Roll = 180)
    // 使得点云坐标系从 "倒装" 恢复为 "正装" (Y轴回正)
    // ==========================================================
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corrected(new pcl::PointCloud<pcl::PointXYZ>);
    
    // 定义变换矩阵：Roll = 180度 (PI)
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    
    // 执行变换
    pcl::transformPointCloud(*cloud, *cloud_corrected, transform);

    // 传入修正后的点云进行处理
    processSingleLineLidar(cloud_corrected, "left", left_calib_queue_, scan_left_viz_pub_);
}

void LidarCalibration::scanRightCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!velo_ready_) return;

    // 1. 转为点云
    sensor_msgs::PointCloud2 cloud_msg;
    projector_.projectLaser(*msg, cloud_msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *cloud);

    // ==========================================================
    // [新增] 倒装修正：绕 X 轴旋转 180 度 (Roll = 180)
    // ==========================================================
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corrected(new pcl::PointCloud<pcl::PointXYZ>);
    
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    
    pcl::transformPointCloud(*cloud, *cloud_corrected, transform);

    // 传入修正后的点云进行处理
    processSingleLineLidar(cloud_corrected, "right", right_calib_queue_, scan_right_viz_pub_);
}

void LidarCalibration::processSingleLineLidar(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                            std::string position_label,
                                            std::deque<Eigen::Matrix4f>& queue,
                                            ros::Publisher& viz_pub)
{
    // 1. 距离滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_filtered->header = cloud->header;
    const float MIN_RANGE = 0.3f;
    for (const auto& p : cloud->points) {
        float range = std::sqrt(p.x * p.x + p.y * p.y);
        if (range > MIN_RANGE) cloud_filtered->push_back(p);
    }
    if (cloud_filtered->size() < 20) return;

    // 2. 提取线段
    std::vector<Line2D> lines = extract2DLines(cloud_filtered);
    if (lines.size() < 2 || lines.size() > 3) return;

    // 3. 排序 (最长的是侧墙，次长的是前墙)
    std::sort(lines.begin(), lines.end(), [](const Line2D& a, const Line2D& b) {
        return a.length > b.length;
    });

    Line2D detected_side_wall = lines[0]; // 最长线
    Line2D detected_front_wall = lines[1]; // 次长线

    // 垂直检查
    if (std::abs(detected_front_wall.normal.dot(detected_side_wall.normal)) > 0.3) return;

    // ========================================================
    // 4. 计算 Yaw (基于前墙)
    // ========================================================
    Eigen::Vector2f n_velo_front_2d(velo_front_.normal.x(), velo_front_.normal.y());
    n_velo_front_2d.normalize();

    float angle_scan = std::atan2(detected_front_wall.normal.y(), detected_front_wall.normal.x());
    float angle_velo = std::atan2(n_velo_front_2d.y(), n_velo_front_2d.x());
    
    float yaw_diff = angle_velo - angle_scan; 
    while(yaw_diff > M_PI) yaw_diff -= 2*M_PI;
    while(yaw_diff < -M_PI) yaw_diff += 2*M_PI;

    Eigen::Matrix3f R_calib = Eigen::AngleAxisf(yaw_diff, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    // ========================================================
    // 5. [关键修正] 智能匹配侧墙 (解决左右翻转问题)
    // ========================================================
    // 将检测到的侧墙法向量旋转到 Global 坐标系
    Eigen::Vector3f n_side_local(detected_side_wall.normal.x(), detected_side_wall.normal.y(), 0.0);
    Eigen::Vector3f n_side_global = R_calib * n_side_local;

    // 判断这个侧墙到底是左墙还是右墙
    // 通过与 Velodyne 的左/右墙法向量点积来判断
    float score_left = n_side_global.dot(velo_left_.normal);
    float score_right = n_side_global.dot(velo_right_.normal);

    // 真实的侧墙数据
    float target_velo_side_dist = 0.0f;
    Eigen::Vector2f target_velo_side_normal;
    int side_color_r=0, side_color_g=0, side_color_b=0;

    // 选取匹配度最高的墙
    if (score_left > score_right) {
        // 匹配到了左墙
        target_velo_side_dist = velo_left_.avg_distance;
        target_velo_side_normal = Eigen::Vector2f(velo_left_.normal.x(), velo_left_.normal.y());
        side_color_r = 255; // Red
    } else {
        // 匹配到了右墙
        target_velo_side_dist = velo_right_.avg_distance;
        target_velo_side_normal = Eigen::Vector2f(velo_right_.normal.x(), velo_right_.normal.y());
        side_color_g = 255; // Green
    }
    target_velo_side_normal.normalize();

    // ========================================================
    // 6. 着色 (用于 RViz 验证)
    // ========================================================
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    auto add_colored_line = [&](const Line2D& line, int r, int g, int b) {
        for (const auto& p : line.points) {
            pcl::PointXYZRGB cp; cp.x=p.x; cp.y=p.y; cp.z=p.z; cp.r=r; cp.g=g; cp.b=b; colored_cloud->push_back(cp);
        }
    };

    add_colored_line(detected_side_wall, side_color_r, side_color_g, side_color_b); // 动态颜色
    add_colored_line(detected_front_wall, 0, 0, 255); // Front: Blue
    if (lines.size() > 2) add_colored_line(lines[2], 100, 100, 100); // Noise: Grey

    // ========================================================
    // 7. 计算平移 (n*t = d_scan - d_velo)
    // ========================================================
    float d_velo_f = velo_front_.avg_distance;
    float d_scan_f = detected_front_wall.distance;
    
    float d_velo_s = target_velo_side_dist;
    float d_scan_s = detected_side_wall.distance;

    Eigen::Matrix2f A;
    A.row(0) = n_velo_front_2d;
    A.row(1) = target_velo_side_normal; // 使用匹配后的法向量
    
    Eigen::Vector2f b;
    b(0) = d_scan_f - d_velo_f; 
    b(1) = d_scan_s - d_velo_s;
    
    Eigen::Vector2f t_xy = A.inverse() * b;

    // 组装结果
    Eigen::Matrix4f T_inst = Eigen::Matrix4f::Identity();
    T_inst.block<3,3>(0,0) = R_calib;
    T_inst(0,3) = t_xy.x();
    T_inst(1,3) = t_xy.y();
    T_inst(2,3) = 0.0f; 

    queue.push_back(T_inst);
    if (queue.size() > SMOOTH_WINDOW_SIZE) queue.pop_front();
    Eigen::Matrix4f T_smooth = getSmoothedResult(queue);

    printFinalResult(T_smooth, (position_label == "left" ? "Scan Left" : "Scan Right"));
    publishTransformedScan(colored_cloud, T_smooth, viz_pub);
}


std::vector<Line2D> LidarCalibration::extract2DLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<Line2D> lines;
    if (cloud->size() < 50) return lines;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
    const float CROP_MARGIN = 0.05f; // 切除两端 25cm

    for (int i = 0; i < 3; ++i) {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.03);
        seg.setInputCloud(cloud_copy);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < 20) break;

        // 获取参数
        float dx = coefficients->values[3];
        float dy = coefficients->values[4];
        Eigen::Vector2f line_dir(dx, dy); line_dir.normalize();
        Eigen::Vector2f line_origin(coefficients->values[0], coefficients->values[1]);

        // 法向量 (-dy, dx)
        Eigen::Vector2f n(-dy, dx); 
        n.normalize();

        // 计算质心
        Eigen::Vector2f centroid(0,0);
        for(int idx : inliers->indices) {
            centroid += Eigen::Vector2f(cloud_copy->points[idx].x, cloud_copy->points[idx].y);
        }
        centroid /= static_cast<float>(inliers->indices.size());

        // [强制] 法向量指向原点
        if (n.dot(centroid) > 0) n = -n;

        // 绝对距离
        float d = std::abs(n.dot(centroid)); 

        // 投影切除
        float min_s = std::numeric_limits<float>::max();
        float max_s = -std::numeric_limits<float>::max();
        std::vector<float> projections;
        projections.reserve(inliers->indices.size());
        
        for (int idx : inliers->indices) {
            const auto& p = cloud_copy->points[idx];
            float s = line_dir.dot(Eigen::Vector2f(p.x, p.y) - line_origin);
            projections.push_back(s);
            if(s < min_s) min_s = s;
            if(s > max_s) max_s = s;
        }

        float valid_min = min_s + CROP_MARGIN;
        float valid_max = max_s - CROP_MARGIN;
        pcl::PointCloud<pcl::PointXYZ> trimmed_points;
        for (size_t k = 0; k < inliers->indices.size(); ++k) {
            float s = projections[k];
            if (s > valid_min && s < valid_max) {
                trimmed_points.push_back(cloud_copy->points[inliers->indices[k]]);
            }
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_copy);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_copy);

        if (trimmed_points.size() < 10) {
            if (cloud_copy->size() < 20) break;
            continue;
        }

        Line2D line;
        line.normal = n;
        line.distance = d;
        line.length = (valid_max > valid_min) ? (valid_max - valid_min) : 0.0f;
        line.found = true;
        line.points = trimmed_points;

        lines.push_back(line);
        if (cloud_copy->size() < 20) break;
    }

    return lines;
}

Eigen::Matrix4f LidarCalibration::getSmoothedResult(const std::deque<Eigen::Matrix4f>& queue)
{
    if (queue.empty()) return Eigen::Matrix4f::Identity();
    Eigen::Vector3f sum_t(0, 0, 0);
    Eigen::Vector4f sum_q(0, 0, 0, 0);
    Eigen::Quaternionf q_ref(queue.front().block<3,3>(0,0));

    for (const auto& T : queue) {
        sum_t += T.block<3,1>(0,3);
        Eigen::Quaternionf q_curr(T.block<3,3>(0,0));
        if (q_curr.dot(q_ref) < 0) q_curr.coeffs() = -q_curr.coeffs();
        sum_q += q_curr.coeffs();
    }
    sum_t /= static_cast<float>(queue.size());
    sum_q.normalize();

    Eigen::Matrix4f T_smooth = Eigen::Matrix4f::Identity();
    T_smooth.block<3,3>(0,0) = Eigen::Quaternionf(sum_q).toRotationMatrix();
    T_smooth.block<3,1>(0,3) = sum_t;
    return T_smooth;
}

void LidarCalibration::publishTransformedScan(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                              const Eigen::Matrix4f& T,
                                              ros::Publisher& pub)
{
    if (cloud->empty()) return;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::transformPointCloud(*cloud, *transformed, T);
    
    sensor_msgs::PointCloud2 out_msg;
    pcl::toROSMsg(*transformed, out_msg);
    out_msg.header.frame_id = lidar_frame_; 
    out_msg.header.stamp = ros::Time::now();
    pub.publish(out_msg);
}

void LidarCalibration::printFinalResult(const Eigen::Matrix4f& T, std::string sensor_name)
{
    Eigen::Vector3f t = T.block<3,1>(0,3);
    Eigen::Vector3f e = T.block<3,3>(0,0).eulerAngles(2,1,0); 

    std::cout << "[" << sensor_name << "] -> [Velodyne] : ";
    std::cout << "x=" << std::fixed << std::setprecision(3) << t.x() << ", ";
    std::cout << "y=" << std::fixed << std::setprecision(3) << t.y() << ", ";
    // std::cout << "yaw=" << std::fixed << std::setprecision(2) << toRadians(e[0]) << " rad" << std::endl;
    std::cout << "yaw=" << std::fixed << std::setprecision(2) << toDegrees(e[0]) << " " << std::endl;
}

void LidarCalibration::run() { ros::spin(); }
float LidarCalibration::toRadians(float d) { return d * M_PI / 180.0f; }
float LidarCalibration::toDegrees(float r) { return r * 180.0f / M_PI; }



1.修改地牛四号车的单线雷达ros驱动，升级四号车的单线雷达固件。
2.排查地牛四号车速腾air的问题，升级补盲雷达固件。
3.检测地牛三号车的单线雷达，排查单线雷达的问题。
4.测试富锐C200单线雷达。

