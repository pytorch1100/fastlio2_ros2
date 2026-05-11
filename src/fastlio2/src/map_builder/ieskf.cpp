#include "ieskf.h"



void State::operator+=(const V21D& delta)
{

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
    delta.block<3, 1>(0, 0) = (inp.gyro_ - m_x.bg) * dt;
    delta.block<3, 1>(3, 0) = 0.5 * (m_x.r_wi * (inp.acc_ - m_x.ba) + g) * dt * dt;     // g前的符号是正号
    delta.block<3, 1>(6, 0);
    delta.block<3, 1>(9, 0);
    delta.block<3, 1>(12, 0) = (m_x.r_wi * (inp.acc_ - m_x.ba) - g) * dt;
    delta.block<3, 1>(15, 0);
    delta.block<3, 1>(18, 0);

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


    // 3 计算矩阵G

    // 4 计算误差状态的协方差矩阵P
}