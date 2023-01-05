#ifndef MUVT_ROS_PLANNER_EXECUTOR_H
#define MUVT_ROS_PLANNER_EXECUTOR_H

// ros
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// XBot and CartesI/O
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

// muvt
#include <muvt_core/planner/dcm_planner.h>
#include <muvt_core/optimizer/optimizer.h>
#include <muvt_core/environment/contact/vertex_contact.h>
#include <muvt_core/environment/contact/edge_collision.h>
#include <muvt_core/environment/contact/edge_relative_pose.h>
#include <muvt_core/environment/contact/edge_multi_relative_poses.h>
#include <muvt_core/environment/contact/edge_steering.h>

// msgs and srvs
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <std_srvs/Empty.h>

namespace Muvt { namespace Planner {

class ContactPlannerExecutor {

public:

    enum CONTACT_MODEL {POINT_CONTACT,SURFACE_CONTACT};

    ContactPlannerExecutor();

    void run();

private:
    // init functions
    void init_load_model();
    void init_load_config();
    void init_interactive_marker();
    void interactive_markers_feedback (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void init_load_cartesian_interface();

    void generate_footsteps();

    // solve IK and execute the trajectory
    void update_vertices_and_edges();

    // publish ROS Markers for footsteps, CP, ZMP and CoM trajectories
    void publish_markers();

    // swing foot start motion condition
    bool check_cp_inside_support_polygon(Eigen::Vector3d cp, Eigen::Affine3d foot_pose);

    // swing foot trajectories
    Eigen::Affine3d swing_trajectory(double time, Eigen::Affine3d x_init, Eigen::Affine3d x_fin, double t_init, double step_time);

    // ROS service definitions
    bool execute_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    ros::NodeHandle _nh, _nhpr;
    ros::ServiceServer _exec_srv;
    ros::Publisher _zmp_pub, _cp_pub, _footstep_name_pub, _footstep_pub, _com_pub;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;

    XBot::ModelInterface::Ptr _model;
    std::shared_ptr<XBot::Cartesian::Utils::RobotStatePublisher> _rspub;
    XBot::Cartesian::CartesianInterfaceImpl::Ptr _ci;

    Eigen::Vector3d _initial_com_pos;
    std::vector<Eigen::Vector3d> _com_trj, _cp_trj;
    std::vector<Muvt::HyperGraph::Contact> _footstep_seq;
    Muvt::Planner::DCMPlanner _planner;

    std::string _home_position_name;
    std::string _world_frame_name;
    std::string _base_frame_name;
    std::vector<std::string> _contact_names;
    std::vector<int> _contact_sequence;
    std::vector<double> _contact_dimensions;
    unsigned int _n_contacts;
    CONTACT_MODEL _contact_model;

    Muvt::HyperGraph::OptimizerContact _g2o_optimizer;

    bool _execute;

    Eigen::Affine3d _tmp_affine3d;
};
} }

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

#define YAML_PARSE(yaml, name, type) \
    if(yaml[#name]) \
{ \
    type value = yaml[#name].as<type>(); \
    std::cout << "Found " #type " option '" #name "' with value = " << value << std::endl; \
    name = value; \
    } \
    else { \
    throw std::runtime_error("No option '" #name "' specified, abort"); \
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

#endif // MUVT_ROS_PLANNER_EXECUTOR_H
