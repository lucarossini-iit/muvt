#ifndef POINT_CLOUD_MANAGER_H
#define POINT_CLOUD_MANAGER_H

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace XBot { namespace HyperGraph { namespace Utils {

class PointCloudManager {
public:
    PointCloudManager(std::string topic_name,
                      ros::NodeHandle nh);

    // Constructor that creates a box centered in a specific position
    PointCloudManager(Eigen::Vector3d pos,
                      ros::NodeHandle);

    void run();

private:
    void callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg);

    ros::NodeHandle _nh;
    ros::Subscriber _pc_sub;
    ros::Publisher _pc_pub;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _point_cloud;
};
} } }
#endif // POINT_CLOUD_MANAGER_H
