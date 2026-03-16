#include "self_calibration_mid/self_calibration_mid.hpp"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_calibration_node");
    ros::NodeHandle nh("~");
    
    LidarCalibration calib(nh);
    std::cout << "PCL版本: " << PCL_VERSION << std::endl;
    ROS_INFO("Starting lidar calibration node...");
    calib.run();
    
    return 0;
}