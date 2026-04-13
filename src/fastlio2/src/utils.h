#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>

/**
 * 用于ros2中的时间戳和double类型的时间戳互相转换
 */
class Utils
{
public:
    static double getSec(std_msgs::msg::Header &header);
};