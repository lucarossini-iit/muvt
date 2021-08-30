#ifndef POINT_CLOUD_MANAGER_H
#define POINT_CLOUD_MANAGER_H

#include <thread>
#include <chrono>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

namespace XBot { namespace HyperGraph { namespace Utils {

class PointCloudManager {
public:
    enum class ExtractorType { OBJECT, EUCLIDEANCLUSTER };

    PointCloudManager(std::string topic_name,
                      ros::NodeHandle nh);

    // Constructor that creates a box centered in a specific position
    PointCloudManager(Eigen::Vector3d pos,
                      ros::NodeHandle);

    void run();

private:
    void callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg);

    void extractObject();
    void clusterExtraction();

    ros::NodeHandle _nh, _nhpr;
    ros::Subscriber _pc_sub;
    ros::Publisher _sc_pub, _mc_pub;
    std::vector<ros::Publisher> _cc_pub;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _point_cloud, _scene_cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _cluster_cloud;

    bool _isCallbackDone;

    std::vector<tf::Transform> _transform;
    tf::TransformBroadcaster _broadcaster;

    ExtractorType _extractor_type;
};
} } }
#endif // POINT_CLOUD_MANAGER_H
