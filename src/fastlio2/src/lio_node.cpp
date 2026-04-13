#include <iostream>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "commons.h"
#include "utils.h"

// 需求，写一个node，订阅mid360发布的激光雷达点云和IMU数据
struct NodeConfig
{
    /* data */
    std::string lidar_topic = "/livox/lidar";
    std::string imu_topic = "/livox/imu";
};

struct StateData
{
    std::mutex imu_buf_mtx;
    std::deque<IMUData> imu_buf;
    double last_imu_timestamp;      // 上一个入buf的imu消息时间戳
};



class LIONode : public rclcpp::Node         // 继承Node类
{
public:
    LIONode() : Node("lio_node")            // 自定义类的构造函数,首先要调用父类的构造函数，传入节点名称，在构造函数的初始化列表中，初始化派生类对象中的基类子对象
    {
        RCLCPP_INFO(this->get_logger(), "LIONode has been started.");

        m_pcd_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            m_node_cfg.lidar_topic,
            10,
            std::bind(&LIONode::lidarCallBack, this, std::placeholders::_1));

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_node_cfg.imu_topic,
            10,
            [this](sensor_msgs::msg::Imu::SharedPtr imu_msg){
                RCLCPP_INFO(this->get_logger(), "recv mid360 imu msg");
            }
        );
    }

    void lidarCallBack(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg)  // 接收到livox的lidar点云数据
    {
        RCLCPP_INFO(this->get_logger(), "recv mid360 lidar pointcloud");
    }

    void imuCallBack(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        RCLCPP_INFO(this->get_logger(), "recv mid360 imu msg");
        std::lock_guard<std::mutex> lock(m_state_data.imu_buf_mtx);

        double time_stamp = Utils::getSec(imu_msg->header);
        // 清除已过期的时间戳
        if (time_stamp < m_state_data.last_imu_timestamp)
        {
            RCLCPP_WARN(this->get_logger(), "imu buf out time, 清理掉再接收新到来的数据");
            std::deque<IMUData>().swap(m_state_data.imu_buf);
        }

        // 将ros格式的imu数据转换
        IMUData imu_data;
        imu_data.acc_  << imu_msg->linear_acceleration.x, 
                    imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z; // 给Eigen::Vector3d对象赋值，一共两种方式
        imu_data.gyro_ = Eigen::Vector3d(imu_msg->angular_velocity.x, 
                    imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
        imu_data.time_ = time_stamp;    // 获取imu数据的时间戳

        m_state_data.imu_buf.emplace_back(imu_data);   // push_back 替换为 emplace_back，避免不必要的复制构造

        m_state_data.last_imu_timestamp = imu_data.time_;
    }

private:
    NodeConfig m_node_cfg;

    StateData m_state_data;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_pcd_sub;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;   // 指向imu订阅者的共享智能指针，订阅imu话题数据
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LIONode> lionode_ptr = std::make_shared<LIONode>();
    rclcpp::spin(lionode_ptr);
    rclcpp::shutdown();
    return 0;
}