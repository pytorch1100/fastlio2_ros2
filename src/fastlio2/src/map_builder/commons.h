#pragma once
#include <eigen3/Eigen/Eigen>

using V3D = Eigen::Vector3d;

template <typename T>
using Vec = std::vector<T>;

struct IMUData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc_;               // 加速度计的测量值
    V3D gyro_;              // 角速度计的测量值
    double time_;           // imu数据采样时刻
    IMUData() = default;    // 默认构造函数
    IMUData(const V3D& acc, const V3D& gyro, const double time) : acc_(acc), gyro_(gyro), time_(time) { }
};
