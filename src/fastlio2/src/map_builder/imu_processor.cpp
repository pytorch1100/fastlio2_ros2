#include "imu_processor.h"

IMUProcessor::IMUProcessor(Config& config, std::shared_ptr<IESKF>& kf)
    : m_config(config), m_kf(kf)
{
    m_Q.setIdentity();
    m_Q.block<3, 3>(0, 0) = config.na * Eigen::Matrix3d::Identity();
    m_Q.block<3, 3>(3, 3) = config.ng * M3D::Identity();
    m_Q.block<3, 3>(6, 6) = config.n_ba * M3D::Identity();
    m_Q.block<3, 3>(9, 9) = config.n_bg * M3D::Identity();
    
    m_imu_cache.clear();
    m_poses_cache.clear();
    m_last_acc.setZero();
    m_last_gyro.setZero();
    m_last_propagate_end_time = 0.0;
}

bool IMUProcessor::initialize(SyncPackage& package)
{
    // 将同步包中的数据放入m_imu_cache
    m_imu_cache.insert(m_imu_cache.end(), package.imu_vec.begin(), package.imu_vec.end());
    if (m_imu_cache.size() < static_cast<size_t>(m_config.imu_init_num))
    {
        std::cout << "m_imu_cache中缓存的imu不够" << std::endl;
        return fasle;
    }

    // 计算平均加速度和平均角速度
    V3D acc_mean, gyro_mean;
    for (auto &imu : m_imu_cache)
    {
        acc_mean += imu.acc_;
        gyro_mean += imu.gyro_;
    }
    acc_mean /= static_cast<double>(m_imu_cache.size());
    gyro_mean /= static_cast<double>(m_imu_cache.size());
}


IMUProcessor::undistort(SyncPackage& package)
{

}