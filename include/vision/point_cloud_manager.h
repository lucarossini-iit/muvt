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
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <teb_test/ObjectMessageString.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <octomap_msgs/OctomapWithPose.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>

namespace XBot { namespace HyperGraph { namespace Utils {

class PointCloudManager {
public:
    typedef std::shared_ptr<PointCloudManager> Ptr;

    PointCloudManager(std::string topic_name,
                      ros::NodeHandle nh);

    void run();

private:
    void callback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg);
    void callback_robot_filtered(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg);
    void octomap_callback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg);

    void voxelFiltering();
    void passThroughFilter();
    void clusterExtraction();
    void outlierRemoval(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    void planarSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    void removePointsBelowPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, pcl::ModelCoefficients::Ptr coefficients);
    void generateObjectMsg();
    void publishObjectMarkers();

    ros::NodeHandle _nh, _nhpr;
    ros::Subscriber _pc_sub, _pc_robot_filtered_sub, _octomap_sub;
    ros::Publisher _pc_voxel_pub, _pc_pass_through_pub, _pc_outlier_pub, _pc_planar_pub, _obj_pub, _ma_pub;
    ros::Publisher _time_pub;
    std::vector<ros::Publisher> _cc_pub;
    std::string _frame_id;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _point_cloud, _cloud_voxel_filtered, _cloud_pass_through_filtered, _cloud_self_robot_filtered, _cloud_planar_segmented, _cloud_without_outliers, _cloud_above_plane;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> _cluster_cloud;

    bool _isCallbackDone, _isCallbackRobotFilteredDone;

    std::vector<tf::Transform> _transforms;
    tf::TransformBroadcaster _broadcaster;
    tf::StampedTransform _transform;
    tf::TransformListener _listener;

    teb_test::ObjectMessageString _objects;

    Eigen::Affine3d _w_T_cam;

    std_msgs::Float64MultiArray _times;
};
} } }
#endif // POINT_CLOUD_MANAGER_H
