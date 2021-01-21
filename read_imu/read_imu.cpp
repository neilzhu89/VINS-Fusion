#include <ros/ros.h>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

#define SAVE_IMU true;

std::string OUTPUT_FOLDER = "/home/jing/catkin_fusion_ws/output/";
std::string IMU_DATA_PATH = OUTPUT_FOLDER + "/imu.csv";
std::string IMU_TOPIC = "/camera/imu";

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    if (SAVE_IMU)
    {
       ofstream imu_sensor_file(IMU_DATA_PATH, ios::app);
       imu_sensor_file.setf(ios::fixed, ios::floatfield);
       imu_sensor_file.precision(0);
       imu_sensor_file << t * 1e9 << ",";
       imu_sensor_file.precision(5);
       imu_sensor_file << dx << ","
              << dy << ","
              << dz << ","
              << rx << ","
              << ry << ","
              << rz << ","
              << endl;
       imu_sensor_file.close();
    }
    return;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_imu");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    std::cout << "imu data path" << IMU_DATA_PATH << std::endl;
    std::ofstream fout(IMU_DATA_PATH, std::ios::out);
    fout.close();

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());

    ros::spin();

    return 0;
}