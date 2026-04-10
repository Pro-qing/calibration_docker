#include "self_calibration_mid/self_calibration_mid.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <clocale>
#include <cmath>
#include <iomanip>

// 构造函数
LidarCalibration::LidarCalibration(ros::NodeHandle& nh) : nh_(nh)
{
    std::setlocale(LC_ALL, ""); 
    
    // 参数读取
    nh_.param<std::string>("lidar_frame", lidar_frame_, "velodyne");
    nh_.param<std::string>("base_frame", base_frame_, "base_link");
    nh_.param<std::string>("mid_frame", mid_frame_, "mid_lidar");
    nh_.param<std::string>("save_path", save_path_, "$(env HOME)/src/self_calibration_mid/param/lidar_calibration.yaml");

    // 读取新的测量参数
    nh_.param<float>("dist_left_wheel_to_front", dist_left_wheel_to_front_, 2.50f);
    nh_.param<float>("dist_right_wheel_to_front", dist_right_wheel_to_front_, 2.55f);
    nh_.param<float>("dist_left_wheel_to_left_wall", dist_left_wheel_to_left_wall_, 1.8f);
    nh_.param<float>("dist_right_wheel_to_right_wall", dist_right_wheel_to_right_wall_, 2.02f);
    nh_.param<float>("wheel_track", wheel_track_, 0.68f);
    nh_.param<float>("manual_lidar_height", manual_lidar_height_, 1.92f);

    cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_16", 1, &LidarCalibration::pointCloudCallback, this);
    mid_cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/points_mid", 1, &LidarCalibration::midPointCloudCallback, this);

    filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("filtered_points", 1);
    wall_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("wall_points", 1);
    
    mid_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("mid_filtered_points", 1);
    mid_wall_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("mid_wall_points", 1);
    transformed_mid_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/velodyne_points_mid", 1);
    
    T_base_to_velo_.setIdentity(); 
    T_base_to_mid_.setIdentity();  
    velo_ready_ = false;
    calibration_queue_.clear();
    
    // 初始化计算Base相对于Room的先验位置
    calculateBaseRoomExtrinsics();

    ROS_INFO("================================================");
    ROS_INFO("   双雷达标定 (可视化验证版 + 车辆Yaw角补偿)");
    ROS_INFO("   车辆 Yaw 偏角: %.2f 度", toDegrees(base_yaw_));
    ROS_INFO("   RViz话题: /velodyne_points_mid (Frame: velodyne)");
    ROS_INFO("================================================");
}

// --------------------------------------------------------------------------------
// 0. Base_Link -> Room 参数预计算 (核心角度与距离补偿)
// --------------------------------------------------------------------------------
void LidarCalibration::calculateBaseRoomExtrinsics()
{
    // 1. 计算车辆相对于墙面的 Yaw 偏角
    // 若左侧离前墙更近 (dl < dr)，说明车头偏向右侧，Yaw角为负。
    float sin_theta = (dist_left_wheel_to_front_ - dist_right_wheel_to_front_) / wheel_track_;
    sin_theta = std::max(-1.0f, std::min(1.0f, sin_theta)); // 防止无效浮点运算
    base_yaw_ = std::asin(sin_theta);

    // 2. 计算Base中心到各墙面的理论垂直距离 (用于构建 Room Frame 坐标系)
    base_to_front_wall_ = (dist_left_wheel_to_front_ + dist_right_wheel_to_front_) / 2.0f;
    
    // 侧墙距离补偿：需考虑车轮由于偏航引起的Y方向偏移分量
    float y_compensation = (wheel_track_ / 2.0f) * std::cos(base_yaw_);
    base_to_left_wall_ = dist_left_wheel_to_left_wall_ + y_compensation;
    base_to_right_wall_ = dist_right_wheel_to_right_wall_ + y_compensation;
}

// --------------------------------------------------------------------------------
// 1. Velodyne -> Base_Link 计算
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

    if (detectThreePlanes(z_filtered, velo_left_, velo_right_, velo_front_, false)) {
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

void LidarCalibration::calculateVeloToBase()
{
    // --- 1. 计算雷达原点在虚拟 Room 坐标系中的位置 ---
    float p_lidar_room_x = base_to_front_wall_ - velo_front_.avg_distance;
    
    float y_from_left = base_to_left_wall_ - velo_left_.avg_distance;
    float y_from_right = -base_to_right_wall_ + velo_right_.avg_distance;
    
    float conf_sum = velo_left_.confidence + velo_right_.confidence;
    float p_lidar_room_y = 0.0f;
    if (conf_sum > 0) {
        p_lidar_room_y = (y_from_left * velo_left_.confidence + y_from_right * velo_right_.confidence) / conf_sum;
    } else {
        p_lidar_room_y = y_from_left;
    }
    
    float p_lidar_room_z = manual_lidar_height_;
    Eigen::Vector3f P_lidar_room(p_lidar_room_x, p_lidar_room_y, p_lidar_room_z);

    // --- 2. 计算 Room 坐标系在 Velodyne 坐标系中的姿态 ---
    // 前墙法向量反向即为 Room 的 X 轴
    Eigen::Vector3f x_room_in_velo = -velo_front_.normal;
    Eigen::Vector3f y_room_in_velo;
    
    if (conf_sum > 0) {
        y_room_in_velo = (-velo_left_.normal * velo_left_.confidence + velo_right_.normal * velo_right_.confidence) / conf_sum;
    } else {
        y_room_in_velo = -velo_left_.normal;
    }

    // 正交化处理
    y_room_in_velo = (y_room_in_velo - (y_room_in_velo.dot(x_room_in_velo)) * x_room_in_velo).normalized();
    Eigen::Vector3f z_room_in_velo = x_room_in_velo.cross(y_room_in_velo).normalized();

    Eigen::Matrix3f R_room_in_velo;
    R_room_in_velo.col(0) = x_room_in_velo;
    R_room_in_velo.col(1) = y_room_in_velo;
    R_room_in_velo.col(2) = z_room_in_velo;
    
    // Velodyne 相对于 Room 的旋转
    Eigen::Matrix3f R_velo_in_room = R_room_in_velo.transpose();

    // -----------------------------------------------------------
    // 关键修正：加入车辆 Yaw 角补偿
    // 构建 Base 在 Room 中的旋转矩阵 (只绕 Z 轴旋转 base_yaw_)
    // -----------------------------------------------------------
    Eigen::Matrix3f R_base_in_room;
    R_base_in_room = Eigen::AngleAxisf(base_yaw_, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    // 最终计算 Velodyne 相对于 Base 的旋转矩阵和平移向量
    // R_velo_in_base = R_room_in_base * R_velo_in_room
    Eigen::Matrix3f R_velo_in_base = R_base_in_room.transpose() * R_velo_in_room;

    // t_velo_in_base = R_room_in_base * P_lidar_room (因为Base原点就是Room原点)
    Eigen::Vector3f t_velo_in_base = R_base_in_room.transpose() * P_lidar_room;

    T_base_to_velo_.setIdentity();
    T_base_to_velo_.block<3,3>(0,0) = R_velo_in_base;
    T_base_to_velo_.block<3,1>(0,3) = t_velo_in_base;
}

// --------------------------------------------------------------------------------
// 2. Mid -> Base_Link 计算 (同样加入 Yaw 补偿)
// --------------------------------------------------------------------------------
void LidarCalibration::midPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!velo_ready_) return;

    // 保留一份原始点云用于变换发布
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *raw_cloud);
    
    if (raw_cloud->empty()) return;

    // 处理用的副本
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(*raw_cloud));

    // 1. 距离滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr dist_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    dist_filtered->header = cloud->header;
    const float MIN_RANGE = 1.0f; 

    for (const auto& p : cloud->points) {
        float range = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
        if (range > MIN_RANGE) {
            dist_filtered->push_back(p);
        }
    }
    if (dist_filtered->empty()) return;
    cloud = dist_filtered; 

    // 2. 地面检测
    if (!detectGround(cloud, mid_ground_)) return;

    // 3. 墙面滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-10.0, mid_ground_.avg_distance - 0.2); 
    pass.filter(*wall_cloud);

    sensor_msgs::PointCloud2 debug_msg;
    pcl::toROSMsg(*wall_cloud, debug_msg);
    debug_msg.header = msg->header;
    mid_filtered_pub_.publish(debug_msg);

    // 4. 三面墙检测 & 标定
    if (detectThreePlanes(wall_cloud, mid_left_, mid_right_, mid_front_, true)) {
        
        calculateMidToBase();
        
        // 计算瞬时外参 T_mid -> velo
        Eigen::Matrix4f T_instant = T_base_to_velo_.inverse() * T_base_to_mid_;
        
        // 加入队列
        calibration_queue_.push_back(T_instant);
        if (calibration_queue_.size() > SMOOTH_WINDOW_SIZE) {
            calibration_queue_.pop_front();
        }

        // 获取平滑结果
        Eigen::Matrix4f T_final = getSmoothedResult();
        
        printFinalResult(T_final);

        // 写入 YAML 文件
        if (calibration_queue_.size() >= SMOOTH_WINDOW_SIZE) {
            saveToYAML(T_final, save_path_);
        }

        // 发布转换后的点云到 RViz
        publishTransformedMidCloud(raw_cloud, T_final);

        // 可视化提取的平面
        pcl::PointCloud<pcl::PointXYZRGB> colored;
        auto add = [&](const LaserLine& l, int r, int g, int b) {
            for(auto p : l.raw_points) {
                pcl::PointXYZRGB cp; cp.x=p.x; cp.y=p.y; cp.z=p.z; 
                uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                cp.rgb = *reinterpret_cast<float*>(&rgb);
                colored.push_back(cp);
            }
        };
        add(mid_ground_, 250,250,0);
        add(mid_left_, 255,0,0);
        add(mid_right_, 0,255,0);
        add(mid_front_, 0,0,255);
        sensor_msgs::PointCloud2 out; pcl::toROSMsg(colored, out);
        out.header = msg->header; mid_wall_pub_.publish(out);
    }
}

void LidarCalibration::calculateMidToBase()
{
    // --- 1. 计算 Mid 雷达在虚拟 Room 坐标系中的位置 ---
    float p_mid_room_x = base_to_front_wall_ - mid_front_.avg_distance;
    
    float y_from_left = base_to_left_wall_ - mid_left_.avg_distance;
    float y_from_right = mid_right_.avg_distance - base_to_right_wall_;
    
    float total_conf = mid_left_.confidence + mid_right_.confidence;
    float p_mid_room_y = 0.0f;
    if (total_conf > 0) {
        p_mid_room_y = (y_from_left * mid_left_.confidence + y_from_right * mid_right_.confidence) / total_conf;
    } else {
        p_mid_room_y = y_from_left; 
    }
    
    float p_mid_room_z = mid_ground_.avg_distance;
    Eigen::Vector3f P_mid_room(p_mid_room_x, p_mid_room_y, p_mid_room_z);

    // --- 2. 计算 Room 坐标系在 Mid 雷达坐标系中的姿态 ---
    Eigen::Vector3f z_room_in_mid = mid_ground_.normal;
    Eigen::Vector3f x_room_in_mid = -mid_front_.normal;
    
    x_room_in_mid = (x_room_in_mid - (x_room_in_mid.dot(z_room_in_mid)) * z_room_in_mid).normalized();
    Eigen::Vector3f y_room_in_mid = z_room_in_mid.cross(x_room_in_mid).normalized();
    
    Eigen::Matrix3f R_room_in_mid;
    R_room_in_mid.col(0) = x_room_in_mid;
    R_room_in_mid.col(1) = y_room_in_mid;
    R_room_in_mid.col(2) = z_room_in_mid;
    
    Eigen::Matrix3f R_mid_in_room = R_room_in_mid.transpose();
    
    // --- 3. 施加 Yaw 角度补偿推导 Base 到 Mid ---
    Eigen::Matrix3f R_base_in_room;
    R_base_in_room = Eigen::AngleAxisf(base_yaw_, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    Eigen::Matrix3f R_mid_in_base = R_base_in_room.transpose() * R_mid_in_room;
    Eigen::Vector3f t_mid_in_base = R_base_in_room.transpose() * P_mid_room;

    T_base_to_mid_.setIdentity();
    T_base_to_mid_.block<3,3>(0,0) = R_mid_in_base;
    T_base_to_mid_.block<3,1>(0,3) = t_mid_in_base;
}

// 点云变换发布函数
void LidarCalibration::publishTransformedMidCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                                 const Eigen::Matrix4f& T_mid_to_velo)
{
    if (cloud->empty()) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, T_mid_to_velo);
    
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_cloud, output_msg);
    output_msg.header.frame_id = lidar_frame_; 
    output_msg.header.stamp = ros::Time::now();
    
    transformed_mid_pub_.publish(output_msg);
}

Eigen::Matrix4f LidarCalibration::getSmoothedResult()
{
    if (calibration_queue_.empty()) return Eigen::Matrix4f::Identity();

    Eigen::Vector3f sum_t(0, 0, 0);
    Eigen::Vector4f sum_q(0, 0, 0, 0);
    Eigen::Quaternionf q_ref(calibration_queue_.front().block<3,3>(0,0));

    for (const auto& T : calibration_queue_) {
        sum_t += T.block<3,1>(0,3);
        Eigen::Quaternionf q_curr(T.block<3,3>(0,0));
        if (q_curr.dot(q_ref) < 0) {
            q_curr.coeffs() = -q_curr.coeffs();
        }
        sum_q += q_curr.coeffs();
    }

    float n = static_cast<float>(calibration_queue_.size());
    sum_t /= n;
    sum_q.normalize();

    Eigen::Matrix4f T_smooth = Eigen::Matrix4f::Identity();
    T_smooth.block<3,3>(0,0) = Eigen::Quaternionf(sum_q).toRotationMatrix();
    T_smooth.block<3,1>(0,3) = sum_t;

    return T_smooth;
}

bool LidarCalibration::detectGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, LaserLine& ground)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cand(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(1.0, 3.0); 
    pass.filter(*cand);
    
    if (cand->size() < 100) return false;

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setInputCloud(cand);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.size() < 100) return false;

    Eigen::Vector3f n(coeff->values[0], coeff->values[1], coeff->values[2]);
    if (n.z() > 0) n = -n; 

    ground.avg_distance = std::abs(coeff->values[3]);
    ground.normal = n; 
    ground.point_count = inliers->indices.size();
    
    ground.raw_points.clear();
    for(int idx : inliers->indices) ground.raw_points.push_back(cand->points[idx]);
    
    return true;
}

bool LidarCalibration::detectThreePlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                        LaserLine& left, LaserLine& right, LaserLine& front,
                                        bool is_mid_lidar)
{
    if (cloud->empty()) return false;
    
    struct Region { std::string label; float min_a; float max_a; };
    std::vector<Region> regions;
    
    if (!is_mid_lidar) {
        regions = { {"front", -45, 45}, {"left", 45, 135}, {"right", -135, -45} };
    } else {
        regions = { {"front", -45, 45}, {"right", 45, 135}, {"left", -135, -45} };
    }

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

void LidarCalibration::printFinalResult(const Eigen::Matrix4f& T)
{
    Eigen::Vector3f t = T.block<3,1>(0,3);
    Eigen::Vector3f e = T.block<3,3>(0,0).eulerAngles(2,1,0); 
    
    std::cout << "\033[2J\033[1;1H"; 
    std::cout << "============================================" << std::endl;
    std::cout << "        EXTRINSIC CALIBRATION RESULT        " << std::endl;
    std::cout << "      Child: Mid  -->  Parent: Velodyne     " << std::endl;
    std::cout << "      Status: Averaging " << calibration_queue_.size() << " / " << SMOOTH_WINDOW_SIZE << " frames" << std::endl;
    std::cout << "============================================" << std::endl;
    std::cout << std::fixed << std::setprecision(4);
    
    std::cout << "Vehicle Yaw Offset to Room: " << toDegrees(base_yaw_) << " deg" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    std::cout << "Translation (m):" << std::endl;
    std::cout << "  x: " << t.x() << std::endl;
    std::cout << "  y: " << t.y() << std::endl;
    std::cout << "  z: " << t.z() << std::endl;
    std::cout << std::endl;
    
    std::cout << "Rotation (Euler RPY in radians):" << std::endl;
    std::cout << "  roll:  " << e[2] << std::endl;
    std::cout << "  pitch: " << e[1] << std::endl;
    std::cout << "  yaw:   " << e[0] << std::endl;
    std::cout << std::endl;
    
    std::cout << "Rotation (Euler RPY in degrees):" << std::endl;
    std::cout << "  roll:  " << toDegrees(e[2]) << " deg" << std::endl;
    std::cout << "  pitch: " << toDegrees(e[1]) << " deg" << std::endl;
    std::cout << "  yaw:   " << toDegrees(e[0]) << " deg" << std::endl;
    std::cout << "============================================" << std::endl;
}

void LidarCalibration::saveToYAML(const Eigen::Matrix4f& T, const std::string& filename)
{
    std::ofstream out(filename);
    if (!out.is_open()) {
        ROS_ERROR_THROTTLE(1.0, "无法打开文件以保存标定结果: %s", filename.c_str());
        return;
    }

    Eigen::Vector3f t = T.block<3,1>(0,3);
    Eigen::Vector3f e = T.block<3,3>(0,0).eulerAngles(2,1,0); 
    Eigen::Quaternionf q(T.block<3,3>(0,0));

    out << std::fixed << std::setprecision(6);
    out << "lidar_extrinsics:\n";
    out << "  parent_frame: \"" << lidar_frame_ << "\"\n";
    out << "  child_frame: \"" << mid_frame_ << "\"\n";
    
    out << "  translation: [x, y, z]\n";
    out << "    x: " << t.x() << "\n";
    out << "    y: " << t.y() << "\n";
    out << "    z: " << t.z() << "\n";
    
    out << "  rotation_euler_rad: [roll, pitch, yaw]\n";
    out << "    roll: " << e[2] << "\n";
    out << "    pitch: " << e[1] << "\n";
    out << "    yaw: " << e[0] << "\n";

    out << "  rotation_euler_deg: [roll, pitch, yaw]\n";
    out << "    roll: " << toDegrees(e[2]) << "\n";
    out << "    pitch: " << toDegrees(e[1]) << "\n";
    out << "    yaw: " << toDegrees(e[0]) << "\n";
    
    out << "  rotation_quaternion: [x, y, z, w]\n";
    out << "    x: " << q.x() << "\n";
    out << "    y: " << q.y() << "\n";
    out << "    z: " << q.z() << "\n";
    out << "    w: " << q.w() << "\n";
    
    out << "  transformation_matrix:\n";
    for (int i = 0; i < 4; ++i) {
        out << "    - [" << T(i,0) << ", " << T(i,1) << ", " << T(i,2) << ", " << T(i,3) << "]\n";
    }

    out.close();
}

float LidarCalibration::toRadians(float d) { return d * M_PI / 180.0f; }
float LidarCalibration::toDegrees(float r) { return r * 180.0f / M_PI; }

void LidarCalibration::run() { ros::spin(); }

