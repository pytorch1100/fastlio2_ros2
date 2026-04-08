#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

// 需求，写一个node，订阅mid360发布的激光雷达点云和IMU数据
struct NodeConfig
{
    /* data */
    std::string lidar_topic = "/livox/lidar";
};



class LIONode : public rclcpp::Node         // 继承Node类
{
public:
    LIONode() : Node("lio_node")            // 自定义类的构造函数,首先要调用父类的构造函数，传入节点名称，在构造函数的初始化列表中，初始化派生类对象中的基类子对象
    {
        RCLCPP_INFO(this->get_logger(), "LIONode has been started.");

        m_pcd_sub = create_subscription<livox_ros_driver2::msg::CustomMsg>(
            m_node_cfg.lidar_topic,
            10,
            std::bind(&LIONode::lidarCallBack, this, std::placeholders::_1));
    }


    void lidarCallBack(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg)  // 接收到livox的lidar点云数据
    {
        RCLCPP_INFO(this->get_logger(), "recv mid360 lidar pointcloud");
    }

private:
    NodeConfig m_node_cfg;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_pcd_sub;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LIONode> lionode_ptr = std::make_shared<LIONode>();
    rclcpp::spin(lionode_ptr);
    rclcpp::shutdown();
    return 0;
}