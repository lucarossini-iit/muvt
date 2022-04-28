#ifndef DCM_PLANNER_H
#define DCM_PLANNER_H

// ROS and stdlib
#include <algorithm>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// muvt
#include <muvt_core/environment/contact/contact.h>

// msg and srv
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace Muvt { namespace HyperGraph { namespace Planner {

class DCMPlanner {

public:
    DCMPlanner();

    void setNumSteps(const unsigned int& n_steps);
    void setZCoM(const double& z_com);
    void setStepTime(const double& step_time);
    void setStepSize(const double& step_size);
    void setdT(const double& dt);
    void setFootsteps(const std::vector<Contact>& footsteps);

    unsigned int getNumSteps() const;
    double getZCoM() const;
    double getStepTime() const;
    double getStepSize() const;
    double getdT() const;

    void generateSteps(const std::vector<Contact>& initial_footsteps);
    void solve();

    void getSolution(std::vector<Contact>& footsteps,
                     std::vector<Eigen::Vector3d>& cp_trj,
                     std::vector<Eigen::Vector3d>& com_trj) const;

private:
    Eigen::Vector3d cp_trajectory(double time, Eigen::Vector3d init, Eigen::Vector3d zmp);
    Eigen::Vector3d com_trajectory(double time, Eigen::Vector3d cp, Eigen::Vector3d init);
    Eigen::Vector3d com_trajectory_from_vel(Eigen::Vector3d cp, Eigen::Vector3d init);

    unsigned int _n_steps;
    unsigned int _n_feet;
    double _z_com;
    double _step_time;
    double _step_size;
    double _dt;

    std::vector<Contact> _footstep_sequence;
    std::vector<std::vector<Eigen::Vector3d>> _cp_trj;
    std::vector<Eigen::Vector3d> _com_trj;

    ros::NodeHandle _nh, _nhpr;
    ros::Publisher _zmp_pub, _cp_pub, _com_pub, _footstep_pub;

};
} } }

#endif // DCM_PLANNER_H
