#pragma
#include <Eigen/Eigen>
#include <sophus/so3.hpp>
#include "commons.h"

using M12D = Eigen::Matrix<double, 12, 12>;     // 定义一个12行 x 12列的方阵 类型
using M21D = Eigen::Matrix<double, 21, 21>;     // 定义一个21行 x 21列的方阵 类型

using V21D = Eigen::Matrix<double, 21, 1>;      // 定义一个21行 x 1列的向量 类型      状态向量
using V12D = Eigen::Matrix<double, 12, 1>;      // 定义一个12行 x 1列的向量 类型      噪声向量

using M21X12D = Eigen::Matrix<double, 21, 12>;  // 定义一个21行 x 12列的矩阵 类型     表示 误差状态对噪声向量的雅可比矩阵


struct Input
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc_;
    V3D gyro_;
    Input() = default;
    Input(V3D& a, V3D& g) : acc_(a), gyro_(g) { }
    Input(double a1, double a2, double a3, double g1, double g2, double g3) :
            acc_(a1, a2, a3), gyro_(g1, g2, g3) { }
};

struct State
{
    M3D r_wi = M3D::Identity();
    V3D t_wi = V3D::Zero();
    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();
    V3D v    = V3D::Zero();
    V3D bg   = V3D::Zero();
    V3D ba   = V3D::Zero();
    V3D g    = V3D(0.0, 0.0, -9.81);

    void operator+=(const V21D& delta);

    // 输出运算符重载函数
    //friend std::ostream& operator<<(std::ostream& os, const State& state);
};


class IESKF
{
public:
    IESKF() = default;

    void predict(const Input& inp, double dt, const M12D& Q);

    void update();

    State& x() { return m_x; }
    M21D& P()   { return m_P; }

private:

    State m_x;      // 状态向量

    M21D m_P;        // 协方差矩阵P
    M12D m_Q;        // 噪声矩阵Q
    M21D m_F;        // 状态转移矩阵F
    M21X12D m_G;     // G
};