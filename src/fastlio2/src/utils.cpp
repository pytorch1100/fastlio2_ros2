#include "utils.h"

double Utils::getSec(std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec);
}

// livox类型的点云到PCL库点云的转换
pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr livox_msg, int filter_num, double min_range, double max_range)     // 将livox类型的点云数据转换成pcl格式
{
    //pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr_pclcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr ptr_cloud = new pcl::PointCloud<pcl::PointXYZINormal>(); 禁止隐式转换

    // 从裸指针直接构造智能指针
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>());

    // 将原始点云降采样
    int points_num = livox_msg->point_num;

    // 预先为cloud分配内存
    cloud->reserve(points_num / filter_num + 1);
    for (int i = 0; i < points_num; i += filter_num)
    {
        if (1)
        {
            // 距离滤波，根据距离过滤掉异常点
            float x = livox_msg->points[i].x;
            float y = livox_msg->points[i].y;
            float z = livox_msg->points[i].z; 

            if (x * x + y * y + z * z < min_range * min_range || x * x + y * y + z * z > max_range * max_range)
            {
                continue;
            }

            pcl::PointXYZINormal p;
            p.x = x;
            p.y = y;
            p.z = z;

            p.curvature = livox_msg->points[i].offset_time / 1000000.0f; // 将offset_time转换为秒，存储在curvature字段中
            p.intensity = static_cast<float>(livox_msg->points[i].reflectivity);

            cloud->push_back(p);
        }
    }
    return cloud;
}