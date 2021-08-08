#include <vision/point_cloud_manager.h>

using namespace XBot::HyperGraph::Utils;

PointCloudManager::PointCloudManager(std::string topic_name, ros::NodeHandle nh):
_nh(nh),
_point_cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
    _pc_sub = _nh.subscribe(topic_name, 1, &PointCloudManager::callback, this);
}

PointCloudManager::PointCloudManager(Eigen::Vector3d pos, ros::NodeHandle nh):
_nh(nh),
_point_cloud(new pcl::PointCloud<pcl::PointXYZ>)
{
    // Advertise PointCloud topic
    _pc_pub = _nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud", 1, true);

    double size = 0.2;
    double res = 0.05;
    unsigned int index = 0;

    _point_cloud->resize(1000);

    // front face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) - size/2;
            _point_cloud->points[index].y = pos(1) - size/2 + i*res;
            _point_cloud->points[index].z = pos(2) - size/2 + k*res;
            index++;
        }
    }

    // back face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) + size/2;
            _point_cloud->points[index].y = pos(1) - size/2 + i*res;
            _point_cloud->points[index].z = pos(2) - size/2 + k*res;
            index++;
        }
    }

    // left face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) - size/2 + i*res;
            _point_cloud->points[index].y = pos(1) - size/2;
            _point_cloud->points[index].z = pos(2) - size/2 + k*res;
            index++;
        }
    }

    // right face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) - size/2 + i*res;
            _point_cloud->points[index].y = pos(1) + size/2;
            _point_cloud->points[index].z = pos(2) - size/2 + k*res;
            index++;
        }
    }

    // top face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) - size/2 + i*res;
            _point_cloud->points[index].y = pos(1) - size/2 + k*res;
            _point_cloud->points[index].z = pos(2) + size/2;
            index++;
        }
    }

    // bottom face
    for (int k = 0; k <= size/res; k++)
    {
        for (int i = 0; i <= size/res; i++)
        {
            _point_cloud->points[index].x = pos(0) - size/2 + i*res;
            _point_cloud->points[index].y = pos(1) - size/2 + k*res;
            _point_cloud->points[index].z = pos(2) - size/2;
            index++;
        }
    }

    _point_cloud->header.frame_id = "world";
}

void PointCloudManager::callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr &msg)
{
    _point_cloud = msg;
}

void PointCloudManager::run()
{
    _pc_pub.publish(_point_cloud);
}
