#pragma once
#include <eigen3/Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using PtType = pcl::PointXYZINormal;     // 自定义 点云库中点类型的别名
using PcdType = pcl::PointCloud<PtType>; // 自定义 点云库中点云类型的别名

using V3D = Eigen::Vector3d;
using M3D = Eigen::Matrix<double, 3, 3>;

template <typename T>
using Vec = std::vector<T>;

struct Config
{
    double na   = 0.01;
    double ng   = 0.01;
    double n_ba = 0.0001;
    double n_bg = 0.0001;

    int imu_init_num = 20;  // 初始化使用，imu足够多时才进行初始化
};

struct IMUData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc_;               // 加速度计的测量值
    V3D gyro_;              // 角速度计的测量值
    double time_;           // imu数据采样时刻
    IMUData() = default;    // 默认构造函数
    IMUData(const V3D& acc, const V3D& gyro, const double time) : acc_(acc), gyro_(gyro), time_(time) { }
};

struct Pose
{

};

struct SyncPackage
{
    // bool pcd_pushed;            // 代表同步包中是否存在点云 0不存在 1存在
    PcdType::Ptr cloud;   // 使用共享的智能指针
    Vec<IMUData> imu_vec; // lidar帧对应的imu帧vector

    double cloud_start_time = 0.0; // lidar帧开始时间戳
    double cloud_end_time = 0.0;   // lidar帧结束时间戳

}; // 记录一帧点云在扫描时间段内，存在的imu帧
