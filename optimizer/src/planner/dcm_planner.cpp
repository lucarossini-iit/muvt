#include <teb_test/planner/dcm_planner.h>

using namespace XBot::HyperGraph::Planner;

DCMPlanner::DCMPlanner():
_nh(""),
_nhpr("~")
{
    _zmp_pub = _nh.advertise<visualization_msgs::MarkerArray>("zmp", 1, true);
    _cp_pub = _nh.advertise<visualization_msgs::MarkerArray>("cp", 1, true);
    _com_pub = _nh.advertise<visualization_msgs::MarkerArray>("com", 1, true);
    _footstep_pub = _nh.advertise<visualization_msgs::MarkerArray>("footstep", 1, true);
}

void DCMPlanner::setNumSteps(const unsigned int n_steps)
{
    _n_steps = n_steps;
}

void DCMPlanner::setStepSize(const double step_size)
{
    _step_size = step_size;
}

void DCMPlanner::setStepTime(const double step_time)
{
    _step_time = step_time;
}

void DCMPlanner::setZCoM(const double z_com)
{
    _z_com = z_com;
}

unsigned int DCMPlanner::getNumSteps() const
{
    return _n_steps;
}

double DCMPlanner::getStepSize() const
{
    return _step_size;
}

double DCMPlanner::getStepTime() const
{
    return _step_time;
}

double DCMPlanner::getZCoM() const
{
    return _z_com;
}

void DCMPlanner::run()
{
    // publish zmp
    visualization_msgs::MarkerArray ma_zmp, ma_cp, ma_footstep, ma_com;
    for (int i = 0; i < _footstep_sequence.size(); i++)
    {
        visualization_msgs::Marker m_zmp;
        m_zmp.header.frame_id = "map";
        m_zmp.header.stamp = ros::Time::now();
        m_zmp.id = i;
        m_zmp.action = visualization_msgs::Marker::ADD;
        m_zmp.type = visualization_msgs::Marker::SPHERE;
        m_zmp.scale.x = 0.01; m_zmp.scale.y = 0.01; m_zmp.scale.z = 0.01;
        m_zmp.color.r = 1; m_zmp.color.g = 0; m_zmp.color.b = 0; m_zmp.color.a = 1;
        m_zmp.pose.position.x = _footstep_sequence[i].state.zmp(0);
        m_zmp.pose.position.y = _footstep_sequence[i].state.zmp(1);
        m_zmp.pose.position.z = _footstep_sequence[i].state.zmp(2);
        ma_zmp.markers.push_back(m_zmp);

//        visualization_msgs::Marker m_cp;
//        m_cp.header.frame_id = "map";
//        m_cp.header.stamp = ros::Time::now();
//        m_cp.id = i;
//        m_cp.action = visualization_msgs::Marker::ADD;
//        m_cp.type = visualization_msgs::Marker::SPHERE;
//        m_cp.scale.x = 0.02; m_cp.scale.y = 0.02; m_cp.scale.z = 0.02;
//        m_cp.color.r = 0; m_cp.color.g = 0; m_cp.color.b = 1; m_cp.color.a = 1;
//        m_cp.pose.position.x = _footstep_sequence[i].state.cp(0);
//        m_cp.pose.position.y = _footstep_sequence[i].state.cp(1);
//        m_cp.pose.position.z = _footstep_sequence[i].state.cp(2);
//        ma_cp.markers.push_back(m_cp);

        visualization_msgs::Marker m_footstep;
        m_footstep.header.frame_id = "map";
        m_footstep.header.stamp = ros::Time::now();
        m_footstep.id = i;
        m_footstep.action = visualization_msgs::Marker::ADD;
        m_footstep.type = visualization_msgs::Marker::CUBE;
        m_footstep.scale.x = 0.2; m_footstep.scale.y = 0.1; m_footstep.scale.z = 0.02;
        m_footstep.color.r = 1; m_footstep.color.g = 1; m_footstep.color.b = 0; m_footstep.color.a = 0.5;
        m_footstep.pose.position.x = _footstep_sequence[i].state.pose.translation().x();
        m_footstep.pose.position.y = _footstep_sequence[i].state.pose.translation().y();
        m_footstep.pose.position.z = _footstep_sequence[i].state.pose.translation().z();
        ma_footstep.markers.push_back(m_footstep);
    }

    for (int i = 0; i < _cp_trj.size(); i++)
    {
        for (int j = 0; j < _cp_trj[i].size(); j++)
        {
            visualization_msgs::Marker m_cp;
            m_cp.header.frame_id = "map";
            m_cp.header.stamp = ros::Time::now();
            m_cp.id = i*_cp_trj[i].size() + j;
            m_cp.action = visualization_msgs::Marker::ADD;
            m_cp.type = visualization_msgs::Marker::SPHERE;
            m_cp.scale.x = 0.01; m_cp.scale.y = 0.01; m_cp.scale.z = 0.01;
            m_cp.color.r = 0; m_cp.color.g = 0; m_cp.color.b = 1; m_cp.color.a = 1;
            m_cp.pose.position.x = _cp_trj[i][j](0);
            m_cp.pose.position.y = _cp_trj[i][j](1);
            m_cp.pose.position.z = _cp_trj[i][j](2);
            ma_cp.markers.push_back(m_cp);
        }
    }

    for (int i = 0; i < _com_trj.size(); i++)
    {
        visualization_msgs::Marker m_com;
        m_com.header.frame_id = "map";
        m_com.header.stamp = ros::Time::now();
        m_com.id = i;
        m_com.action = visualization_msgs::Marker::ADD;
        m_com.type = visualization_msgs::Marker::SPHERE;
        m_com.scale.x = 0.01; m_com.scale.y = 0.01; m_com.scale.z = 0.01;
        m_com.color.r = 1; m_com.color.g = 0; m_com.color.b = 1; m_com.color.a = 1;
        m_com.pose.position.x = _com_trj[i](0);
        m_com.pose.position.y = _com_trj[i](1);
        m_com.pose.position.z = _com_trj[i](2);
        ma_com.markers.push_back(m_com);
    }

    _zmp_pub.publish(ma_zmp);
    _cp_pub.publish(ma_cp);
    _com_pub.publish(ma_com);
    _footstep_pub.publish(ma_footstep);
}

void DCMPlanner::GenerateSteps(int n_steps)
{
    // init steps
    Contact left_foot("l_sole"), right_foot("r_sole");
    left_foot.state.pose.translation() << 0.0, 0.1, 0.0;
    right_foot.state.pose.translation() << 0.0, -0.1, 0.0;

    // at the moment, always start walking with the right foot
    for (int i = 0; i < n_steps; i++)
    {
        right_foot.state.pose.translation().x() += _step_size;
        left_foot.state.pose.translation().x() += _step_size;

        (i % 2 == 0) ? _footstep_sequence.push_back(right_foot) : _footstep_sequence.push_back(left_foot);
    }
}

Eigen::Vector3d DCMPlanner::cp_trajectory(double time, Eigen::Vector3d init, Eigen::Vector3d zmp)
{
    double omega = std::sqrt(9.81 / _z_com);
    Eigen::Vector3d csi_d = exp(omega * time) * init + (1 - exp(omega * time)) * zmp;

    return csi_d;
}

Eigen::Vector3d DCMPlanner::com_trajectory(double time, Eigen::Vector3d cp, Eigen::Vector3d init)
{
    double omega = std::sqrt(9.81 / _z_com);
    double a = exp(omega * time) / (2*exp(omega * time) - 1);
    double b = (1 - exp(omega * time)) / (2*exp(omega * time) - 1);

    Eigen::Vector3d com = a * init - b * cp;
    com(2) = _z_com;

    return com;
}

Eigen::Vector3d DCMPlanner::com_trajectory_from_vel(Eigen::Vector3d cp, Eigen::Vector3d init, double dt)
{
    double omega = std::sqrt(9.81 / _z_com);

    Eigen::Vector3d x_old = init;
    std::cout << "x_old: " << x_old.transpose() << std::endl;
    std::cout << "cp: " << cp.transpose() << std::endl;
    Eigen::Vector3d x_new = dt * (-omega * x_old + omega * cp) + x_old;
    x_old = x_new;
    std::cout << "x_new: " << x_new.transpose() << std::endl;

    return x_new;
}

void DCMPlanner::ComputeZMPandCP()
{
    // start from the last step and move backwards to the first one.
    // follows the work done in 'Integration of vertical COM motion and angular momentum in an extended Capture Point tracking controller for bipedal walking'
    _footstep_sequence.back().state.zmp = _footstep_sequence.back().state.pose.translation();
    _footstep_sequence.back().state.cp = _footstep_sequence.back().state.pose.translation();

    for(int i = _footstep_sequence.size()-2; i >= 0; i--)
    {
        _footstep_sequence[i].state.zmp = _footstep_sequence[i].state.pose.translation();
        _footstep_sequence[i].state.cp = _footstep_sequence[i].state.zmp + (_footstep_sequence[i+1].state.cp - _footstep_sequence[i].state.zmp) / exp(sqrt(9.81/_z_com)*_step_time);
    }

    // set the initial com position on the first zmp;
    _com_trj.push_back(Eigen::Vector3d(_footstep_sequence[0].state.cp(0), _footstep_sequence[0].state.cp(1), _z_com));

    // compute CP desired trajectory
    double dt = 0.01;
    double time = 0.0;
    _cp_trj.resize(_footstep_sequence.size() - 1);

    for (int ind = 0; ind < _footstep_sequence.size()-1; ind++)
    {
        time = 0;
        while (time < _step_time)
        {
            auto cp = cp_trajectory(time, _footstep_sequence[ind].state.cp, _footstep_sequence[ind].state.zmp);
            _cp_trj[ind].push_back(cp);
            time += dt;
        }
    }

    time = 0;
    auto com_old = _com_trj.back();
    for (int i = 0; i < _cp_trj.size(); i++)
    {
        for (int j = 0; j < _cp_trj[i].size(); j++)
        {
//            _com_trj.push_back(com_trajectory(time, _cp_trj[i][j], _com_trj[0]));
//            time += dt;
            _com_trj.push_back(com_trajectory_from_vel(_cp_trj[i][j], com_old, dt));
            com_old = _com_trj.back();
        }
    }
    std::cout << "final time: " << time << std::endl;
    std::cout << "cp trajectory size: " << _cp_trj.size() * _cp_trj[0].size() << std::endl;
    std::cout << "com trajectory size: " << _com_trj.size() << std::endl;
}

