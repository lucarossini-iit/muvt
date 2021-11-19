#ifndef POINT_CLOUD_MANAGER_H
#define POINT_CLOUD_MANAGER_H

#include <thread>
#include <chrono>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
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
#include <pcl/filters/statistical_outlier_removal.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <teb_test/ObjectMessageString.h>
#include <visualization_msgs/MarkerArray.h>

namespace XBot { namespace HyperGraph { namespace Utils {

class PointCloudManager {
public:
    typedef std::shared_ptr<PointCloudManager> Ptr;

    PointCloudManager(std::string topic_name,
                      ros::NodeHandle nh);

    void run();

private:
    void callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& msg);

    void clusterExtraction();
    void outlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    void generateObjectMsg();
    void publishObjectMarkers();

    ros::NodeHandle _nh, _nhpr;
    ros::Subscriber _pc_sub;
    ros::Publisher _pc_pub, _pc_filt_pub, _pc_filt2_pub, _obj_pub, _ma_pub;
    std::vector<ros::Publisher> _cc_pub;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _point_cloud, _cloud_filtered, _cloud_filtered2;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _cluster_cloud;

    bool _isCallbackDone;

    std::vector<tf::Transform> _transforms;
    tf::TransformBroadcaster _broadcaster;
    tf::StampedTransform _transform;
    teb_test::ObjectMessageString _objects;

    Eigen::Affine3d _w_T_cam;
};
} } }
#endif // POINT_CLOUD_MANAGER_H
