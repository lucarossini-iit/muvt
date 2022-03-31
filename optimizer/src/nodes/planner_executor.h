#ifndef PLANNER_EXECUTOR_H
#define PLANNER_EXECUTOR_H

// ros
#include <ros/ros.h>

// XBot and CartesI/O
#include <XBotInterface/ModelInterface.h>
#include <XBotInterface/RobotInterface.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>

// teb_test
#include <teb_test/planner/dcm_planner.h>

// msgs and srvs
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>

namespace XBot { namespace HyperGraph { namespace Planner {

class PlannerExecutor {

public:
    PlannerExecutor();

    void run();

private:
    // private member functions
    void init_load_model();
    void init_load_config();
    void init_load_cartesian_interface();
    void plan();
    void publish_markers();
    Eigen::Affine3d swing_trajectory(double time, Eigen::Affine3d x_init, Eigen::Affine3d x_fin);

    // ROS service definitions
    bool execute_service(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    ros::NodeHandle _nh, _nhpr;
    ros::ServiceServer _exec_srv;
    ros::Publisher _zmp_pub, _cp_pub, _footstep_pub, _com_pub;

    ModelInterface::Ptr _model;
    RobotInterface::Ptr _robot;
    std::shared_ptr<Cartesian::Utils::RobotStatePublisher> _rspub;
    Cartesian::CartesianInterfaceImpl::Ptr _ci;

    std::vector<Eigen::Vector3d> _com_trj, _cp_trj;
    std::vector<Contact> _footstep_seq;
    DCMPlanner _planner;

    bool _execute;
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
