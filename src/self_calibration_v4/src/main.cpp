#include "self_calibration_v4/self_calibration_v4.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_self_calibration_v4_node");
    ros::NodeHandle nh("~");

    LidarCalibration app(nh);
    app.run();

    return 0;
}