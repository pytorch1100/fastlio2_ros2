#include <iostream>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "map_builder/commons.h"
#include "utils.h"

using PtType = pcl::PointXYZINormal;     // 自定义 点云库中点类型的别名
using PcdType = pcl::PointCloud<PtType>; // 自定义 点云库中点云类型的别名

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
    double last_imu_timestamp; // 上一个入buf的imu消息时间戳

    std::mutex pcd_buf_mtx;
    std::deque<std::pair<double, PcdType::Ptr>> pcd_buf;
    double last_lidar_timestamp;

    bool pcd_pushed;
};

struct SyncPackage
{
    // bool pcd_pushed;            // 代表同步包中是否存在点云 0不存在 1存在
    PcdType::Ptr cloud;   // 使用共享的智能指针
    Vec<IMUData> imu_vec; // lidar帧对应的imu帧vector

    double cloud_start_time = 0.0; // lidar帧开始时间戳
    double cloud_end_time = 0.0;   // lidar帧结束时间戳

}; // 记录一帧点云在扫描时间段内，存在的imu帧

class LIONode : public rclcpp::Node // 继承Node类
{
public:
    LIONode() : Node("lio_node") // 自定义类的构造函数,首先要调用父类的构造函数，传入节点名称，在构造函数的初始化列表中，初始化派生类对象中的基类子对象
    {
        RCLCPP_INFO(this->get_logger(), "LIONode has been started.");

        m_pcd_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            m_node_cfg.lidar_topic,
            10,
            std::bind(&LIONode::lidarCallBack, this, std::placeholders::_1));

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_node_cfg.imu_topic,
            10,
            [this](sensor_msgs::msg::Imu::SharedPtr imu_msg)
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
                imu_data.acc_ << imu_msg->linear_acceleration.x,
                    imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z; // 给Eigen::Vector3d对象赋值，一共两种方式
                imu_data.gyro_ = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                                 imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
                imu_data.time_ = time_stamp; // 获取imu数据的时间戳

                m_state_data.imu_buf.emplace_back(imu_data); // push_back 替换为 emplace_back，避免不必要的复制构造

                m_state_data.last_imu_timestamp = imu_data.time_;
            });

        // m_timer = this->create_wall_timer();
    }

    /**
     * 提取一个lidar帧中的所有imu帧，
     */
    bool syncPackage()
    {
        // 1. 判断imu和lidar缓存中是否存在数据
        if (m_state_data.imu_buf.empty() || m_state_data.pcd_buf.empty())
        {
            return false;
        }

        if (m_state_data.pcd_pushed == false) // 首先同步包中需要有一包lidar帧
        {
            std::lock_guard<std::mutex> mtx(m_state_data.pcd_buf_mtx);
            m_sync_package.cloud = m_state_data.pcd_buf.front().second;

            // 对该lidar帧排序
            std::sort(m_sync_package.cloud->begin(), m_sync_package.cloud->end(), [](PtType &p1, PtType &p2)
                      { return p1.curvature < p2.curvature; });

            m_sync_package.cloud_start_time = m_state_data.pcd_buf.front().first; // 整个lidar帧的起始时间戳
            m_sync_package.cloud_end_time = m_sync_package.cloud_start_time +
                                            m_sync_package.cloud->points.back().curvature / 1000.0; // lidar帧结束时间戳
            m_state_data.pcd_pushed = true;                                                         // lidar点云已经push
        }

        if (m_state_data.last_imu_timestamp < m_sync_package.cloud_end_time)
        {
            return false;
        }

        // 2. 提取对应的imu帧
        Vec<IMUData>().swap(m_sync_package.imu_vec);

        while (!m_state_data.imu_buf.empty() && m_state_data.last_imu_timestamp < m_sync_package.cloud_end_time)
        {
            // 将imu队列中的imu帧放到同步包中
            m_sync_package.imu_vec.emplace_back(m_state_data.imu_buf.front());
            m_state_data.imu_buf.pop_back();    // 从队列中弹出
        }

        // 3 lidar队列中的点云帧弹出
        m_state_data.pcd_buf.pop_back();
        m_state_data.pcd_pushed = false;
    }

    // void syncPackageWwz()
    // {
    //     if (m_state_data.imu_buf.empty() || m_state_data.pcd_buf.empty())
    //     {
    //         return;
    //     }

    //     // 到这一步，表示buffer中都存在数据
    //     if (!m_sync_package.pcd_pushed)     // 同步包中不存在点云，需要将pcd_buf_mtx中的数据推进去
    //     {
    //         std::lock_guard<std::mutex> lock(m_state_data.pcd_buf_mtx);
    //         m_sync_package.cloud = m_state_data.pcd_buf.front().second;
    //         // 对cloud中的点云进行排序，按照时间戳从小到大排序

    //         std::sort(m_sync_package.cloud->points.begin(), m_sync_package.cloud->points.end(), [this](const PtType& pt1, const PtType& pt2){
    //             return pt1.curvature < pt2.curvature;
    //         });
    //         m_state_data.pcd_buf.pop_front();   // 从队列中移除该点云帧
    //         m_sync_package.pcd_pushed = false;      //
    //     }

    //     // 提取imu数据帧
    //     double pcd_start_time = m_sync_package.cloud->header.stamp
    //                     + m_sync_package.cloud->points.front().curvature;      // lidar帧中第一个点的开始时间
    //     double pcd_end_time = m_sync_package.cloud->header.stamp
    //                     + m_sync_package.cloud->points.back().curvature;        // lidar帧中最后一个点的结束时间 end()不可解引用

    //     for (auto& imu : m_state_data.imu_vec)  // 遍历std::deque容器
    //     {
    //         if (imu.time_ < pcd_start_time)   // 找到lidar帧开始点右边的第一个imu帧
    //             continue;

    //         if (imu.time_ < pcd_end_time)       // 找到lidar帧结束点左边的imu帧
    //         {
    //             m_sync_package.imu_vec.emplace_back(imu);
    //             m_state_data.imu_vec.pop_front();   // 从队列中移除该imu帧
    //         }
    //     }

    //     //
    // }

    void lidarCallBack(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg) // 接收到livox的lidar点云数据
    {
        RCLCPP_INFO(this->get_logger(), "recv mid360 lidar pointcloud");
        std::lock_guard<std::mutex> lock(m_state_data.pcd_buf_mtx);

        // 调用点云类型转换函数，进行转换
        PcdType::Ptr ptr_cloud = Utils::livox2pcl(livox_msg, 4, 1, 100);

        // 判断当前buf中的点云数据是否过期
        double t = Utils::getSec(livox_msg->header);
        if (t < m_state_data.last_lidar_timestamp)
        {
            RCLCPP_WARN(this->get_logger(), "lidar data is outdate");
            std::deque<std::pair<double, PcdType::Ptr>>().swap(m_state_data.pcd_buf); // 清理buf中已过期的数据
        }

        m_state_data.pcd_buf.emplace_back(std::pair<double, PcdType::Ptr>(t, ptr_cloud));
        m_state_data.last_lidar_timestamp = t;
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
        imu_data.acc_ << imu_msg->linear_acceleration.x,
            imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z; // 给Eigen::Vector3d对象赋值，一共两种方式
        imu_data.gyro_ = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                         imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
        imu_data.time_ = time_stamp; // 获取imu数据的时间戳

        m_state_data.imu_buf.emplace_back(imu_data); // push_back 替换为 emplace_back，避免不必要的复制构造

        m_state_data.last_imu_timestamp = imu_data.time_;
    }

    void timerCallBack()
    {
        syncPackage();
    }

private:
    NodeConfig m_node_cfg;

    StateData m_state_data;

    SyncPackage m_sync_package;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_pcd_sub;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub; // 指向imu订阅者的共享智能指针，订阅imu话题数据

    rclcpp::TimerBase::SharedPtr m_timer; // 实例化一个定时器
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LIONode> lionode_ptr = std::make_shared<LIONode>();
    rclcpp::spin(lionode_ptr);
    rclcpp::shutdown();
    return 0;
}