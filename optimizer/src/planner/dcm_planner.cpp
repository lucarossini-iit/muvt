#include <teb_test/planner/dcm_planner.h>

using namespace XBot::HyperGraph::Planner;

DCMPlanner::DCMPlanner():
_nh(""),
_nhpr("~")
{}

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

void DCMPlanner::generateSteps()
{
    // init steps
    Contact left_foot("l_sole"), right_foot("r_sole");
    left_foot.state.pose.translation() << 0.0, 0.1, 0.0;
    right_foot.state.pose.translation() << 0.0, -0.1, 0.0;

    // at the moment, always start walking with the right foot
    for (int i = 0; i < _n_steps; i++)
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
    Eigen::Vector3d x_new = dt * (-omega * init + omega * cp) + init;

    return x_new;
}

void DCMPlanner::solve()
{
    _cp_trj.clear();
    _com_trj.clear();

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
}


void DCMPlanner::getSolution(std::vector<Contact>& footsteps, std::vector<Eigen::Vector3d>& cp_trj, std::vector<Eigen::Vector3d>& com_trj) const
{
    footsteps = _footstep_sequence;

    for(auto i : _cp_trj)
    {
        for (auto j : i)
        {
            cp_trj.push_back(j);
        }
    }

    com_trj = _com_trj;
}
