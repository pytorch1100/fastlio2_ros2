#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>

/**
 * 用于ros2中的时间戳和double类型的时间戳互相转换
 */
class Utils
{
public:
    static double getSec(std_msgs::msg::Header &header);
    static pcl::PointCloud<pcl::PointXYZINormal>::Ptr livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg, int filter_num, double min_range, double max_range);
};