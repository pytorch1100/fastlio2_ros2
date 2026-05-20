#include "ieskf.h"

double State::gravity = 9.81;

M3D Jr(const V3D& inp)
{
    return Sophus::SO3d::leftJacobian(inp).trnaspose();
}

M3D jrInv(const V3D& inp)
{
    return Sopuhs::SO3d::leftJacobianInverse(inp).transpose();
}



void State::operator+=(const V21D& delta)
{
    r_wi *=  Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    t_wi += delta.segment<3>(3);
    r_il *= Sopuhs::SO3d::exp(delta.segment<3>(6)).matrix();
    t_il += delta.segment<3>(9);
    v += delta.segment<3>(12);
    bg += delta.segment<3>(15);
    ba += delta.segment<3>(18);
}

V21D State::operator-(const State& other) const
{
    V21D delta;
    delta.setZero();

    delta.segment<3>(0) = Sopuhs::SO3d(other.r_wi.transpose() * r_wi).log();
    delta.segment<3>(3) = t_wi - other.t_wi;
    delta.segment<3>(6) = Sophus::SO3d(other.r_il.transpose() * r_il).log();
    delta.segment<3>(9) = t_il - other.t_il;
    delta.segment<3>(12) = v - other.v;
    delta.segment<3>(15) = bg - other.bg;
    delta.segment<3>(18) = ba - other.ba;

    return delta;
}

// << 二元运算符重载函数
// std::ostream& operator<<(std::ostream& os, const State& state)
// {

// }



// ESKF相关成员函数
void IESKF::predict(const Input& inp, const double &dt, const M12D& Q)
{
    /**
     * @brief 预测过程步骤
     * 1. 中值积分计算名义状态变化量和名义状态
     * 2. 计算离散运动方程相对于误差状态F和噪声的雅可比矩阵G
     * 3. 误差状态协方差矩阵更新
     * 
     */
    V21D delta = V21D::Zero(); // 这里需要实现中值积分计算名义状态变化量delta

    // 速度： a * dt
    // 位置： v * dt + 0.5 * a * dt * dt      a_mea = a + ba - r_wi.transpose() * g.   a = a_mea - bg + r_wi.transpose() * g
    // a_w = r_wi * (a_mea - bg) + g
    // 旋转矩阵或四元数： 角速度 * dt
    // q p q p v ba bg 状态量的顺序
    // delta.block<3, 1>(0, 0) = (inp.gyro_ - m_x.bg) * dt;
    // delta.block<3, 1>(3, 0) = 0.5 * (m_x.r_wi * (inp.acc_ - m_x.ba) + m_x.g) * dt * dt;     // g前的符号是正号
    // delta.block<3, 1>(6, 0);
    // delta.block<3, 1>(9, 0);
    // delta.block<3, 1>(12, 0) = (m_x.r_wi * (inp.acc_ - m_x.ba) + m_x.g) * dt;
    // delta.block<3, 1>(15, 0);
    // delta.block<3, 1>(18, 0);

    delta.segment<3>(1) = (inp.gyro_ - m_x.bg) * dt;    //旋转增量
    delta.segment<3>(3) = m_x.v * dt;                   //位移增量
    delta.segment<3>(12) = (m_x.r_wi * (inp.acc_ - m_x.ba) + m_x.g) * dt;   // 速度增量


    /**
     * @brief 比力f：物体除了重力外受到的力
     * a 物体加速度
     * g 重力
     * f = a - g
     * 自由落体时，加速度为g，比力为0
     * 静止时，加速度为0，比力为-g
     * 
     */

    // 2 计算状态转移矩阵F
    m_F.setIdentity();
    m_F.block<3, 3>(0, 0) = Sophus::SO3d::exp(-(inp.gyro_ - m_x.bg) * dt).matrix();
    m_F.block<3, 3>(0, 15) = -Jr((inp.gyro_ - m_x.bg) * dt) * dt;
    m_F.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity() * dt;
    m_F.block<3, 3>(12, 0) = -m_x.r_wi * Sophus::SO3d::hat(inp.acc_ - m_x.ba) * dt;
    m_F.block<3, 3>(12, 18) = -m_x.r_wi * dt;

    // 3 计算矩阵G
    m_G.setZero();
    m_G.block<3, 3>(0, 0) = -Jr((inp.gyro_ - m_x.bg) * dt) * dt;
    m_G.block<3, 3>(12, 3) = -m_x.r_wi * dt;
    m_G.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

    // 4 计算误差状态的协方差矩阵P
    m_x += delta;
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
}