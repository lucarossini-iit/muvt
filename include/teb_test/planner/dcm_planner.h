#ifndef DCM_PLANNER_H
#define DCM_PLANNER_H

// ROS and stdlib
#include <algorithm>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// teb_test
#include <environment/contact/contact.h>

// msg and srv
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace XBot { namespace HyperGraph { namespace Planner {

class DCMPlanner {

public:
    DCMPlanner();

    void GenerateSteps(int n_steps);
    void ComputeZMPandCP();

    void run();

private:
    void init_load_config();
    Eigen::Vector3d cp_trajectory(double time, Eigen::Vector3d init, Eigen::Vector3d zmp);
    Eigen::Vector3d com_trajectory(double time, Eigen::Vector3d cp, Eigen::VectorXd init);

    double _z_com;
    double _T_step;
    double _step_size;

    std::vector<Contact> _footstep_sequence;
    std::vector<std::vector<Eigen::Vector3d>> _cp_trj;
    std::vector<Eigen::Vector3d> _com_trj;

    ros::NodeHandle _nh, _nhpr;
    ros::Publisher _zmp_pub, _cp_pub, _com_pub, _footstep_pub;

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

#endif // DCM_PLANNER_H
