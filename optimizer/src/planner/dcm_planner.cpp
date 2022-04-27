#include <muvt/planner/dcm_planner.h>

#define GRAVITY 9.81

using namespace Muvt::HyperGraph::Planner;

DCMPlanner::DCMPlanner():
_nh(""),
_nhpr("~")
{}

void DCMPlanner::setNumSteps(const unsigned int& n_steps)
{
    _n_steps = n_steps;
}

void DCMPlanner::setStepSize(const double& step_size)
{
    _step_size = step_size;
}

void DCMPlanner::setStepTime(const double& step_time)
{
    _step_time = step_time;
}

void DCMPlanner::setZCoM(const double& z_com)
{
    _z_com = z_com;
}

void DCMPlanner::setdT(const double& dt)
{
    _dt = dt;
}

void DCMPlanner::setFootsteps(const std::vector<Contact>& footsteps)
{
    _footstep_sequence.clear();
    _footstep_sequence = footsteps;
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

double DCMPlanner::getdT() const
{
    return _dt;
}

void DCMPlanner::generateSteps(const std::vector<Contact>& initial_footsteps)
{

    _n_feet = initial_footsteps.size();
    std::vector<Contact> current_steps = initial_footsteps;

//    _footstep_sequence = current_steps;
    for (int i = 0; i < _n_feet; i++)
    {
        std::vector<Contact>::iterator it = std::min_element(current_steps.begin(), current_steps.end());
        int index = it - current_steps.begin();
        _footstep_sequence.push_back(current_steps[index]);
        current_steps.erase(it);
    }

    current_steps = _footstep_sequence;

    // TODO: manage orientation
    for (unsigned int i = 0; i < _n_steps; i++)
    {
        unsigned int next_foot = i % _n_feet;
        if (_n_feet == 2 && (i == 0 || i == _n_steps - 1))
            current_steps[next_foot].state.pose.translation().x() += _step_size / 2;
        else
            current_steps[next_foot].state.pose.translation().x() += _step_size;
        current_steps[next_foot].state.pose.linear().setIdentity();
        current_steps[next_foot].state.time += _step_time;

        _footstep_sequence.push_back(current_steps[next_foot]);
    }
}

Eigen::Vector3d DCMPlanner::cp_trajectory(double time, Eigen::Vector3d init, Eigen::Vector3d zmp)
{
    double omega = std::sqrt(GRAVITY / _z_com);
    Eigen::Vector3d csi_d = exp(omega * time) * init + (1 - exp(omega * time)) * zmp;

    return csi_d;
}

Eigen::Vector3d DCMPlanner::com_trajectory(double time, Eigen::Vector3d cp, Eigen::Vector3d init)
{
    double omega = std::sqrt(GRAVITY / _z_com);
    double a = exp(omega * time) / (2*exp(omega * time) - 1);
    double b = (1 - exp(omega * time)) / (2*exp(omega * time) - 1);

    Eigen::Vector3d com = a * init - b * cp;
    com(2) = _z_com;

    return com;
}

Eigen::Vector3d DCMPlanner::com_trajectory_from_vel(Eigen::Vector3d cp, Eigen::Vector3d init)
{
    double omega = std::sqrt(GRAVITY / _z_com);
    Eigen::Vector3d x_new = _dt * (-omega * init + omega * cp) + init;
    x_new(2) = _z_com;

    return x_new;
}

void DCMPlanner::solve()
{
    _cp_trj.clear();
    _com_trj.clear();

    // initialize first zmp in the middle of the starting feet position
    double avg_x = 0.0;
    double avg_y = 0.0;
    for(unsigned int i = 0; i < _n_feet; i++) // Take the first n since they are the initial footsteps
    {
      avg_x = _footstep_sequence[i].state.pose.translation().x() + avg_x;
      avg_y = _footstep_sequence[i].state.pose.translation().y() + avg_y;
    }
    avg_x = avg_x / _n_feet;
    avg_y = avg_y / _n_feet;
    _footstep_sequence[0].state.zmp << avg_x, avg_y, 0;
    _footstep_sequence[0].state.cp = _footstep_sequence[0].state.zmp;

    // start from the last step and move backwards
    avg_x = 0.0;
    avg_y = 0.0;
    for(unsigned int i = 0; i < _n_feet; i++) // Take the last n since they are the final footsteps
    {
      avg_x = (*(_footstep_sequence.end() - (i+1))).state.pose.translation().x() + avg_x;
      avg_y = (*(_footstep_sequence.end() - (i+1))).state.pose.translation().y() + avg_y;
    }
    avg_x = avg_x / _n_feet;
    avg_y = avg_y / _n_feet;
    _footstep_sequence.back().state.zmp  << avg_x, avg_y, 0;
    _footstep_sequence.back().state.cp = _footstep_sequence.back().state.zmp;

    for(int i = _footstep_sequence.size() - 2; i > 0; i--)
    {
        _footstep_sequence[i].state.zmp = _footstep_sequence[i].state.pose.translation();
        _footstep_sequence[i].state.cp = _footstep_sequence[i].state.zmp + (_footstep_sequence[i+1].state.cp - _footstep_sequence[i].state.zmp) / exp(sqrt(GRAVITY/_z_com)*_step_time);
    }

    _footstep_sequence[0].state.zmp = (_footstep_sequence[1].state.cp - std::exp(std::sqrt(GRAVITY / _z_com) * _step_time) * _footstep_sequence[0].state.cp) / (1 - std::exp(std::sqrt(GRAVITY / _z_com) * _step_time));

    // set the initial com position on the first zmp;
    _com_trj.push_back(Eigen::Vector3d(_footstep_sequence[0].state.cp(0), _footstep_sequence[0].state.cp(1), _z_com));

    // compute CP desired trajectory
    double time = 0.0;

    _cp_trj.resize(_footstep_sequence.size() - 1);

    for (int ind = 0; ind < _footstep_sequence.size()-1; ind++)
    {
        time = 0;
        while (time < _step_time)
        {
            auto cp = cp_trajectory(time, _footstep_sequence[ind].state.cp, _footstep_sequence[ind].state.zmp);
            _cp_trj[ind].push_back(cp);
            time += _dt;
        }
    }

//    time = 0;
    auto com_old = _com_trj.back();
    for (unsigned int i = 0; i < _cp_trj.size(); i++)
    {
        for (unsigned int j = 0; j < _cp_trj[i].size(); j++)
        {
//            _com_trj.push_back(com_trajectory(time, _cp_trj[i][j], _com_trj[0]));
//            time += _dt;
            _com_trj.push_back(com_trajectory_from_vel(_cp_trj[i][j], com_old));
            com_old = _com_trj.back();
        }
    }

}


void DCMPlanner::getSolution(std::vector<Contact>& footsteps, std::vector<Eigen::Vector3d>& cp_trj, std::vector<Eigen::Vector3d>& com_trj) const
{
    footsteps.clear();
    cp_trj.clear();
    com_trj.clear();

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
