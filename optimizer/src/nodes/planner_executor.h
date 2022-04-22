#ifndef PLANNER_EXECUTOR_H
#define PLANNER_EXECUTOR_H

// ros
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// muvt
#include <muvt/planner/dcm_planner.h>
#include <muvt/optimizer/optimizer.h>
#include <muvt/environment/contact/vertex_contact.h>
#include <muvt/environment/contact/edge_collision.h>
#include <muvt/environment/contact/edge_relative_pose.h>
#include <muvt/environment/contact/edge_steering.h>

// msgs and srvs
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>

namespace Muvt { namespace HyperGraph { namespace Planner {

class PlannerExecutor {

public:
    PlannerExecutor();

    void run();

private:
    // init functions
    void init_load_config();
    void init_interactive_marker();
    void interactive_markers_feedback (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    // solve IK and execute the trajectory
    void plan();

    // publish ROS Markers for footsteps, CP, ZMP and CoM trajectories
    void publish_markers();

    // swing foot start motion condition
    bool check_cp_inside_support_polygon(Eigen::Vector3d cp, Eigen::Affine3d foot_pose);

    // swing foot trajectories
    Eigen::Affine3d swing_trajectory(double time, Eigen::Affine3d x_init, Eigen::Affine3d x_fin, double t_init, double step_time);

    ros::NodeHandle _nh, _nhpr;
    ros::Publisher _zmp_pub, _cp_pub, _footstep_pub, _com_pub;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;

    std::vector<Eigen::Vector3d> _com_trj, _cp_trj;
    std::vector<Contact> _footstep_seq;
    DCMPlanner _planner;

    OptimizerContact _g2o_optimizer;
};
} } }

/* Macro for option parsing */
#define YAML_PARSE_OPTION(yaml, name, type, default_value) \
    type name = default_value; \
    if(yaml[#name]) \
{ \
    type value = yaml[#name].as<type>(); \
    std::cout << "Found " #type " option '" #name "' with value = " << value << std::endl; \
    name = value; \
    } \
    else { \
    std::cout << "No option '" #name "' specified, using default" << std::endl; \
    } \
    /* End macro for option parsing */

namespace std
{

template <typename T>
inline std::ostream& operator<<(std::ostream& os, std::vector<T> v)
{
    os << "\n";
    for(const auto& elem : v)
    {
        os << " - " << elem << "\n";
    }

    return os;
}

inline std::ostream& operator<<(std::ostream& os, std::list<std::string> v)
{
    os << "\n";
    for(const auto& elem : v)
    {
        os << " - " << elem << "\n";
    }

    return os;
}
}

#endif // PLANNER_EXECUTOR_H
