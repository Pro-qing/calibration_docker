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

    // 墙面距离参数
    nh_.param<float>("actual_left_distance", actual_left_dist_, 0.0f);
    nh_.param<float>("actual_right_distance", actual_right_dist_, 0.0f);
    nh_.param<float>("actual_front_distance", actual_front_dist_, 0.0f);

    // 安装高度参数
    nh_.param<float>("manual_lidar_height", manual_lidar_height_, 1.95f);
    nh_.param<float>("manual_left_laser_height", manual_left_height_, 0.1f);
    nh_.param<float>("manual_right_laser_height", manual_right_height_, 0.1f);

    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_16", 1, &LidarCalibration::pointCloudCallback, this);
    scan_left_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_left_rqt", 1, &LidarCalibration::scanLeftCallback, this);
    scan_right_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan_right", 1, &LidarCalibration::scanRightCallback, this);

    filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", 1);
    wall_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("wall_points", 1);
    
    scan_left_viz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_scan_left", 1);
    scan_right_viz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_scan_right", 1);

    T_base_to_velo_.setIdentity();
    velo_ready_ = false;

    // 初始化显示状态
    T_final_left_ = Eigen::Matrix4f::Identity();
    T_final_right_ = Eigen::Matrix4f::Identity();
    has_left_data_ = false;
    has_right_data_ = false;

    left_calib_queue_.clear();
    right_calib_queue_.clear();

    // 初始清屏
    std::cout << "\033[2J"; 
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
        add(velo_left_, 255,0,0); add(velo_right_, 0,255,0); add(velo_front_, 0,0,255);
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
        
        // [关键] 强制法向量指向原点
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
// 2. 单线雷达标定逻辑 (倒装修正 -> 智能匹配 -> TF合成)
// --------------------------------------------------------------------------------

void LidarCalibration::scanLeftCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!velo_ready_) return;
    sensor_msgs::PointCloud2 cloud_msg;
    projector_.projectLaser(*msg, cloud_msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *cloud);

    // 1. 倒装修正: Roll = 180 (绕X旋转)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corrected(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f T_flip = Eigen::Affine3f::Identity();
    T_flip.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*cloud, *cloud_corrected, T_flip);

    processSingleLineLidar(cloud_corrected, "left", left_calib_queue_, scan_left_viz_pub_);
}

void LidarCalibration::scanRightCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if (!velo_ready_) return;
    sensor_msgs::PointCloud2 cloud_msg;
    projector_.projectLaser(*msg, cloud_msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(cloud_msg, *cloud);

    // 1. 倒装修正: Roll = 180 (绕X旋转)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corrected(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Affine3f T_flip = Eigen::Affine3f::Identity();
    T_flip.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
    pcl::transformPointCloud(*cloud, *cloud_corrected, T_flip);

    processSingleLineLidar(cloud_corrected, "right", right_calib_queue_, scan_right_viz_pub_);
}

void LidarCalibration::processSingleLineLidar(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                            std::string position_label,
                                            std::deque<Eigen::Matrix4f>& queue,
                                            ros::Publisher& viz_pub)
{
    // 距离滤波 (30cm)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_filtered->header = cloud->header;
    const float MIN_RANGE = 0.3f;
    for (const auto& p : cloud->points) {
        float range = std::sqrt(p.x * p.x + p.y * p.y);
        if (range > MIN_RANGE) cloud_filtered->push_back(p);
    }
    if (cloud_filtered->size() < 20) return;

    // 提取线段
    std::vector<Line2D> lines = extract2DLines(cloud_filtered);
    if (lines.size() < 2 || lines.size() > 3) return;

    // 排序
    std::sort(lines.begin(), lines.end(), [](const Line2D& a, const Line2D& b) {
        return a.length > b.length;
    });

    Line2D detected_side_wall = lines[0]; 
    Line2D detected_front_wall = lines[1]; 

    // 垂直检查
    if (std::abs(detected_front_wall.normal.dot(detected_side_wall.normal)) > 0.3) return;

    // --- A. 计算 Yaw ---
    Eigen::Vector2f n_velo_front_2d(velo_front_.normal.x(), velo_front_.normal.y());
    n_velo_front_2d.normalize();

    float angle_scan = std::atan2(detected_front_wall.normal.y(), detected_front_wall.normal.x());
    float angle_velo = std::atan2(n_velo_front_2d.y(), n_velo_front_2d.x());
    
    float yaw_diff = angle_velo - angle_scan; 
    while(yaw_diff > M_PI) yaw_diff -= 2*M_PI;
    while(yaw_diff < -M_PI) yaw_diff += 2*M_PI;

    Eigen::Matrix3f R_align = Eigen::AngleAxisf(yaw_diff, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    // --- B. 智能匹配侧墙 ---
    Eigen::Vector3f n_side_local(detected_side_wall.normal.x(), detected_side_wall.normal.y(), 0.0);
    Eigen::Vector3f n_side_global = R_align * n_side_local;

    float score_left = n_side_global.dot(velo_left_.normal);
    float score_right = n_side_global.dot(velo_right_.normal);

    float target_velo_side_dist = 0.0f;
    Eigen::Vector2f target_velo_side_normal;
    int r=0, g=0, b=0;

    if (score_left > score_right) {
        target_velo_side_dist = velo_left_.avg_distance;
        target_velo_side_normal = Eigen::Vector2f(velo_left_.normal.x(), velo_left_.normal.y());
        r=255; // Red = Left Wall
    } else {
        target_velo_side_dist = velo_right_.avg_distance;
        target_velo_side_normal = Eigen::Vector2f(velo_right_.normal.x(), velo_right_.normal.y());
        g=255; // Green = Right Wall
    }
    target_velo_side_normal.normalize();

    // --- 可视化数据准备 ---
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    auto add_line = [&](const Line2D& line, int cr, int cg, int cb) {
        for (const auto& p : line.points) {
            pcl::PointXYZRGB cp; cp.x=p.x; cp.y=p.y; cp.z=p.z; cp.r=cr; cp.g=cg; cp.b=cb; colored_cloud->push_back(cp);
        }
    };
    add_line(detected_side_wall, r, g, b);
    add_line(detected_front_wall, 0, 0, 255); // Blue = Front Wall

    // --- C. 计算 Translation (X, Y) ---
    float d_velo_f = velo_front_.avg_distance;
    float d_scan_f = detected_front_wall.distance;
    float d_scan_s = detected_side_wall.distance;

    Eigen::Matrix2f A;
    A.row(0) = n_velo_front_2d;
    A.row(1) = target_velo_side_normal;
    
    Eigen::Vector2f bb;
    bb(0) = d_scan_f - d_velo_f; 
    bb(1) = d_scan_s - target_velo_side_dist;
    
    Eigen::Vector2f t_xy = A.inverse() * bb;

    // --- D. 计算 Translation (Z) ---
    float t_z = 0.0f;
    if (position_label == "left") t_z = manual_left_height_ - manual_lidar_height_;
    else t_z = manual_right_height_ - manual_lidar_height_;

    // --- E. 合成最终变换矩阵 ---
    Eigen::Matrix4f T_align = Eigen::Matrix4f::Identity();
    T_align.block<3,3>(0,0) = R_align;
    T_align(0,3) = t_xy.x();
    T_align(1,3) = t_xy.y();
    T_align(2,3) = t_z;

    Eigen::Matrix4f T_flip = Eigen::Matrix4f::Identity();
    T_flip.block<3,3>(0,0) = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()).toRotationMatrix();

    Eigen::Matrix4f T_total = T_align * T_flip;

    // 平滑
    queue.push_back(T_total);
    if (queue.size() > SMOOTH_WINDOW_SIZE) queue.pop_front();
    Eigen::Matrix4f T_smooth = getSmoothedResult(queue);

    // --- 更新状态并显示 ---
    if (position_label == "left") {
        T_final_left_ = T_smooth;
        has_left_data_ = true;
    } else {
        T_final_right_ = T_smooth;
        has_right_data_ = true;
    }
    displayCombinedStatus();
    
    // 可视化: 将点云转回 Velodyne 坐标系 (T_align 变换)
    publishTransformedScan(colored_cloud, getSmoothedResult(queue) * T_flip.inverse(), viz_pub);
}

// 提取2D线段
std::vector<Line2D> LidarCalibration::extract2DLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    std::vector<Line2D> lines;
    if (cloud->size() < 50) return lines;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
    const float CROP_MARGIN = 0.25f;

    for (int i = 0; i < 3; ++i) {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.05);
        seg.setInputCloud(cloud_copy);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < 20) break;
        float dx = coefficients->values[3];
        float dy = coefficients->values[4];
        Eigen::Vector2f line_dir(dx, dy); line_dir.normalize();
        Eigen::Vector2f line_origin(coefficients->values[0], coefficients->values[1]);

        Eigen::Vector2f n(-dy, dx); n.normalize();
        Eigen::Vector2f centroid(0,0);
        for(int idx : inliers->indices) centroid += Eigen::Vector2f(cloud_copy->points[idx].x, cloud_copy->points[idx].y);
        centroid /= static_cast<float>(inliers->indices.size());

        if (n.dot(centroid) > 0) n = -n;
        float d = std::abs(n.dot(centroid)); 

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

// --------------------------------------------------------------------------------
// 统一显示函数 (刷屏不闪烁)
// --------------------------------------------------------------------------------
void LidarCalibration::displayCombinedStatus()
{
    std::cout << "\033[H"; // 移动光标到左上角

    std::cout << "========================================================" << std::endl;
    std::cout << "   Lidar Auto Calibration Status (Dual Stream)          " << std::endl;
    std::cout << "   Parent Frame: " << std::left << std::setw(20) << lidar_frame_ << std::endl;
    std::cout << "========================================================" << std::endl;

    if (has_left_data_) {
        printMatrixBlock(T_final_left_, "Scan LEFT");
    } else {
        std::cout << "\n[Scan LEFT] : Waiting for data...\n" << std::endl;
        std::cout << std::string(6, '\n'); 
    }

    std::cout << "--------------------------------------------------------" << std::endl;

    if (has_right_data_) {
        printMatrixBlock(T_final_right_, "Scan RIGHT");
    } else {
        std::cout << "\n[Scan RIGHT] : Waiting for data...\n" << std::endl;
        std::cout << std::string(6, '\n'); 
    }

    std::cout << "========================================================" << std::endl;
    std::cout << "\033[0J" << std::flush; // 清除剩余
}

void LidarCalibration::printMatrixBlock(const Eigen::Matrix4f& T, const std::string& label)
{
    Eigen::Vector3f t = T.block<3,1>(0,3);
    Eigen::Quaternionf q(T.block<3,3>(0,0));
    Eigen::Vector3f e = T.block<3,3>(0,0).eulerAngles(0,1,2); // RPY

    std::cout << "[" << label << "]" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  Translation (m) : " 
              << "x=" << std::setw(8) << t.x() 
              << "y=" << std::setw(8) << t.y() 
              << "z=" << std::setw(8) << t.z() << std::endl;

    std::cout << "  Rotation (Rad)  : "
              << "r=" << std::setw(8) << e[0] 
              << "p=" << std::setw(8) << e[1] 
              << "y=" << std::setw(8) << e[2] << std::endl;
              
    std::cout << "  Quaternion      : "
              << "x=" << std::setw(7) << q.x() 
              << "y=" << std::setw(7) << q.y() 
              << "z=" << std::setw(7) << q.z() 
              << "w=" << std::setw(7) << q.w() << std::endl;
}

void LidarCalibration::run() { ros::spin(); }
float LidarCalibration::toRadians(float d) { return d * M_PI / 180.0f; }
float LidarCalibration::toDegrees(float r) { return r * 180.0f / M_PI; }

