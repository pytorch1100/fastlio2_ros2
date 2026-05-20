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

    // 初始化卡尔曼滤波器的状态
    IMUProcessor::m_kf->get_state_x().r_il = m_config.r_il;
    IMUProcessor::m_kf->get_state_x().t_il = m_config.t_il;
    IMUProcessor::m_kf->get_state_x().bg = gyro_mean;

    // 设置重力方向，和初始姿态
    if (m_config.gravity_align == true)
    {
        IMUProcessor::m_kf->get_state_x().r_wi = 
            (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
        IMUProcessor::m_kf->get_state_x().initGravityDir(V3D(0, 0, -1.0));
    }
    else
    {
        IMUProcessor::m_kf->get_state_x().initGravityDir(-acc_mean);
    }

    m_kf->m_P.setIdentity();
    m_kf->m_P.block<3, 3>(6, 6) = 0.00001 * M3D::Identity();
    m_kf->m_P.block<3, 3>(9, 9) = 0.00001 * M3D::Identity();
    m_kf->m_P.block<3, 3>(15, 15) = 0.0001 * M3D::Identity();
    m_kf->m_P.block<3, 3>(18, 18) = 0.0001 * M3D::Identity();
    
    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = package.cloud_end_time;
    return true;
}

// 利用imu计算的pose为对应的点云帧去畸变
IMUProcessor::undistort(SyncPackage& package)
{
    m_imu_cache.clear();
    m_imu_cache.push_back(m_last_imu);
    m_imu_cache.insert(m_imu_cache.end(), package.imu_vec.begin(), package.imu_vec.end());
    
    const double imu_time_end = m_imu_cache.back().time_;

    const double cloud_time_begin = package.cloud_start_time;
    const double propagate_time_end = package.cloud_end_time;
    
    m_poses_cache.clear();
    m_poses_cache.emplace_back(0.0, m_last_acc, m_last_gyro, 
        m_kf->get_state_x().v, m_kf->get_state_x().t_wi, m_kf->get_state_x().r_wi);
    
    V3D acc_mean, gyro_mean;
    double dt = 0.0;
    Input inp;
    inp.acc_ = m_imu_cache.back().acc_;
    inp.gyro_ = m_imu_cache.back().gyro_;

    // 1. 前向传播，计算每一个imu帧的pose，存放到m_pose_cache中
    for (auto it_imu = m_imu_cache.begin(); it_imu < (m_imu_cache.end() - 1); ++it_imu)
    {
        IMUData& head = *(it_imu);
        IMUData& tail = *(it_imu + 1);

        if (tail.time_ < m_last_propagate_end_time)
            continue;

        acc_mean =  0.5 * (head->acc_ + tail->acc_);
        gyro_mean = 0.5 * (head->gyro_ + tail->gyro_);

        if (head.time_ < m_last_propagate_end_time)
            dt = tail.time_ - m_last_propagate_end_time;
        else
            dt = tail.time_ - head.time_;

        inp.acc_ = acc_mean;
        inp.gyro_ = gyro_mean;
        m_kf->predict(inp, dt, m_Q);

        m_last_acc = m_kf->get_state_x().r_wi*(acc_mean - m_kf->get_state_x().ba) + m_kf->get_state_x().g; // 转换到world系下
        m_last_gyro = gyro_mean - m_kf->get_state_x().bg;
        double offset = tail.time_ - cloud_time_begin;
        m_poses_cache.emplace_back(offset, m_last_acc, m_last_gyro, 
            m_kf->get_state_x().v, m_kf->get_state_x().t_wi, m_kf->get_state_x().r_wi);
    }

    dt = propagate_time_end - imu_time_end;
    m_kf->predict(inp, dt, m_Q);
    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = propagate_time_end;

    // 1.记录当前时刻的位姿（前向传播到当前时刻）、记录package中最后一个点对应的iterator
    M3D cur_r_wi = m_kf->get_state_x().r_wi;    // 预测后，当前时刻的旋转矩阵
    V3D cur_t_wi = m_kf->get_state_x().t_wi;    // 预测后，当前时刻imu坐标系在世界坐标系下的坐标，使用vector表示
    M3D cur_r_il = m_kf->get_state_x().r_il;
    V3D cur_t_il = m_kf->get_state_x().t_wi;

    auto it_point = package.cloud->end() - 1;   // lidar帧最后一个点

    // 2.大循环：从后向前，遍历pose cache，取出两个imu帧对应的pose
    for (auto it_pose = m_poses_cache.end() - 1; it_pose > m_poses_cache.begin(); it_pose--)
    {   
        Pose& head = *(it_pose - 1);    // 第一个imu帧对应的pose
        Pose& tail = &it_pose;          // 第二个imu帧对应的pose

        M3D imu_r_wi = head.rot;
        V3D imu_t_wi = head.trans;
        V3D imu_v_wi = head.vel;
        V3D imu_acc = tail.acc;
        V3D imu_gyro = tail.gyro;

        // 3.小循环：从后向前遍历所有package中的所有点，进行去畸变
        for (; it_point >= package.cloud->end(); it_point--)
        {
            // (0) 计算lidar点时刻，相对于head imu帧的时间offset
            double dt =  it_point->curvature / double(1000) - head.offset;

            //（1）计算lidar点时刻，imu在世界坐标系的旋转矩阵和位移
            M3D point_r_wi = imu_r_wi * Sophus::SO3::exp(imu_gyro * dt).matrix();
            V3D point_t_wi = imu_t_wi + imu_v_wi * dt + 0.5 * imu_acc * dt * dt;

            // (2)计算lidar点在运动补偿后，在lidar坐标系下的表示
            V3D L_point_LP = V3D::Zero();   // 表示Lidar坐标系中，lidar坐标系原点到lidar点P的平移
            L_point_LP.x() = it_point->x;   
            L_point_LP.y() = it_point->y;
            L_point_LP.z() = it_point->z;    //补偿前lidar点的坐标，pcl转换成eigen格式

            //(2-1) lidar to imu,获得lidar在imu坐标系下的表示,
            V3D I_trans_IP  = V3D::Zero();  //表示坐标系Imu中，IMU坐标系（坐标系原点I）到lidar点P的平移
            I_trans_IP = cur_r_il * L_point_LP + cur_t_il;

            //(2-2)获得World坐标系下，World坐标系原点到lidar点P的平移，时刻是点P对应的采样时间
            V3D W_trans_WP = V3D::Zero();   // World坐标系下，World坐标系原点到lidar点P的平移
            W_trans_WP = point_r_wi * I_trans_IP + point_t_wi;

            // (3-3)求点P在当前时刻imu坐标系下的表示，统一将所有点反向传播到当前时刻，使用cur_r_wi
            V3D curI_trans_IP =  cur_r_wi.transpose() * W_trans_WP - cur_t_wi;
            // (4-4)最后将点转换到lidar坐标系中
            V3D  L_trans_LP = cur_r_il.transpose() * curI_trans_IP - cur_r_il;


            //V3D p_compensate = cur_r_il.transpose() * (cur_r_wi.transpose() * (point_rot * (cur_r_il * point + cur_t_il) + point_pos - cur_t_wi) - cur_t_il); 
            //L_trans_LP = cur_r_il.transpose() * (cur_r_wi.transpose() * (point_r_wi * (cur_r_il * L_point_LP + cur_t_il) + point_t_wi) - cur_t_wi) - cur_r_il;
            it_point->x = L_trans_LP(0);
            it_point->y = L_trans_LP(1);
            it_point->z = L_trans_LP(2); // 修改packgae点云的坐标，利用补偿后的坐标赋值
            if (it_point == package.cloud->begin())
                break;
        }
    }
    
}