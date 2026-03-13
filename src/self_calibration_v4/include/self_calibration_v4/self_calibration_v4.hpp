#ifndef SELF_CALIBRATION_V4_HPP
#define SELF_CALIBRATION_V4_HPP

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <laser_geometry/laser_geometry.h>
#include <vector>
#include <string>
#include <deque>

// 3D 平面结构体 (用于多线雷达)
struct LaserLine {
    float avg_distance;
    Eigen::Vector3f normal;
    std::string label;
    float confidence;
    int point_count;
    std::vector<pcl::PointXYZ> raw_points;
    LaserLine() : avg_distance(0), confidence(0), point_count(0) {}
};

// 2D 线段结构体 (用于单线雷达)
struct Line2D {
    Eigen::Vector2f normal;     // 法向量 (x, y)
    float distance;             // 原点到直线的垂直距离
    float length;               // 线段有效长度
    bool found;
    pcl::PointCloud<pcl::PointXYZ> points; // 线段包含的点 (用于可视化)
    Line2D() : distance(0), length(0), found(false) {}
};

class LidarCalibration
{
public:
    LidarCalibration(ros::NodeHandle& nh);
    void run();

private:
    ros::NodeHandle nh_;
    
    // --- 订阅者 ---
    ros::Subscriber cloud_sub_;         
    ros::Subscriber scan_left_sub_;     
    ros::Subscriber scan_right_sub_;    

    // --- 发布者 ---
    ros::Publisher filtered_pub_;       
    ros::Publisher wall_points_pub_;    
    ros::Publisher scan_left_viz_pub_;  
    ros::Publisher scan_right_viz_pub_; 

    // --- 工具 ---
    laser_geometry::LaserProjection projector_;

    // --- 参数 ---
    std::string lidar_frame_, base_frame_;
    float actual_left_dist_, actual_right_dist_, actual_front_dist_;
    
    // 高度参数
    float manual_lidar_height_;
    float manual_left_height_;
    float manual_right_height_;

    // --- 状态变量 ---
    Eigen::Matrix4f T_base_to_velo_;
    bool velo_ready_;

    // --- 结果存储 (用于同步显示) ---
    Eigen::Matrix4f T_final_left_;
    Eigen::Matrix4f T_final_right_;
    bool has_left_data_;
    bool has_right_data_;

    // --- 平滑队列 ---
    const size_t SMOOTH_WINDOW_SIZE = 50;
    std::deque<Eigen::Matrix4f> left_calib_queue_;
    std::deque<Eigen::Matrix4f> right_calib_queue_;

    // --- 墙面数据 ---
    LaserLine velo_left_, velo_right_, velo_front_;

    // --- 回调函数 ---
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void scanLeftCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void scanRightCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    // --- 核心处理 ---
    void calculateVeloToBase();
    
    void processSingleLineLidar(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                                std::string position_label,
                                std::deque<Eigen::Matrix4f>& queue,
                                ros::Publisher& viz_pub);

    std::vector<Line2D> extract2DLines(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    bool detectThreePlanes(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           LaserLine& left, LaserLine& right, LaserLine& front);

    Eigen::Matrix4f getSmoothedResult(const std::deque<Eigen::Matrix4f>& queue);

    // --- 显示与可视化 ---
    void displayCombinedStatus();
    void printMatrixBlock(const Eigen::Matrix4f& T, const std::string& label);
    
    void publishTransformedScan(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                                const Eigen::Matrix4f& T,
                                ros::Publisher& pub);

    float toRadians(float d);
    float toDegrees(float r);
};

#endif