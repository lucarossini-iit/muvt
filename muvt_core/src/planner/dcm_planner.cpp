#include <muvt_core/planner/dcm_planner.h>

#define GRAVITY 9.81

using namespace Muvt::HyperGraph::Planner;

DCMPlanner::DCMPlanner():
_w_R_l(Eigen::Matrix3d::Identity())
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

void DCMPlanner::setWalkingDirection(Eigen::Vector2d xy)
{
    xy.normalize();
    double yaw = std::atan2(xy(1), xy(0));
    Eigen::Quaternion<double> q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    _w_R_l = q.toRotationMatrix();
}

void DCMPlanner::generateSteps(const std::vector<Contact>& initial_footsteps)
{
    clear();

    // move to sagittal plane
    Eigen::Vector3d step = _w_R_l * Eigen::Vector3d(_step_size, 0, 0);

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
        {
            current_steps[next_foot].state.pose.translation().x() += step(0) / 2;
            current_steps[next_foot].state.pose.translation().y() += step(1) /2;
        }
        else
        {
            current_steps[next_foot].state.pose.translation().x() += step(0);
            current_steps[next_foot].state.pose.translation().y() += step(1);
        }
        current_steps[next_foot].state.pose.linear() = _w_R_l;
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

    // check if the footstep sequence has to be increased or decreased in size
    check_footstep_sequence();

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

void DCMPlanner::check_footstep_sequence()
{
    for (int i = _n_feet + 1; i < _footstep_sequence.size() - _n_feet - 1; i++)
    {
        std::vector<Contact> contact_succ, contact_prev;
//        if (i < _footstep_sequence.size() - _n_feet && i >_n_feet)
//        {
            std::vector<Contact> c_succ(_footstep_sequence.begin() + i + 1, _footstep_sequence.begin() + i + 1 + _n_feet);
            std::vector<Contact> c_prev(_footstep_sequence.begin() + i - 1 -_n_feet, _footstep_sequence.begin() + i - 1);
            contact_succ = c_succ;
            contact_prev = c_prev;
//        }
//        else if (i >= _footstep_sequence.size() - _n_feet)
//        {
//            std::vector<Contact> c_succ(_footstep_sequence.begin() + i + 1, _footstep_sequence.end());
//            std::vector<Contact> c_prev(_footstep_sequence.begin() + i - 1 -_n_feet, _footstep_sequence.begin() + i - 1);
//            contact_succ = c_succ;
//            contact_prev = c_prev;

//        }
//        else if (i <= _n_feet)
//        {
//            std::vector<Contact> c_succ(_footstep_sequence.begin() + i + 1, _footstep_sequence.begin() + i + 1 + _n_feet);
//            std::vector<Contact> c_prev(_footstep_sequence.begin(), _footstep_sequence.begin() + i);
//            contact_succ = c_succ;
//            contact_prev = c_prev;

//        }


        Contact next_contact;
        for(auto c : contact_succ)
        {
            if(c.getDistalLink() == _footstep_sequence[i].getDistalLink())
            {
                next_contact = c;
                auto it = std::find(contact_succ.begin(), contact_succ.end(), c);
                contact_succ.erase(it);
                break;
            }
        }

        double distance = (_footstep_sequence[i].state.pose.translation() - next_contact.state.pose.translation()).norm();
//        std::cout << distance << std::endl;
        int index = 1;

        // add footsteps whenever required
        if (distance > _step_size * 2)
        {
            for (auto c : contact_succ)
            {
//                std::cout << "OLD OTHER CONTACT: " << std::endl;
//                c.print();

                auto is_same_distal_link = [c](Contact contact)
                {
                    return contact.getDistalLink() == c.getDistalLink();
                };
                auto it_prev = std::find_if(contact_prev.begin(), contact_prev.end(), is_same_distal_link);
                if (it_prev == contact_prev.end())
                    throw std::runtime_error("error");

//                std::cout << "PREVIOUS POSE:" << std::endl;
//                it_prev->print();

                Contact new_contact = *(it_prev);
                new_contact.state.pose.translation().x() = (c.state.pose.translation().x() + new_contact.state.pose.translation().x())/2;
                new_contact.state.pose.translation().y() = (c.state.pose.translation().y() + new_contact.state.pose.translation().y())/2;

//                std::cout << "NEW_CONTACT:" << std::endl;
//                new_contact.print();
//                std::cout << "inserting in position " << i+index << std::endl;

                _footstep_sequence.insert(_footstep_sequence.begin() + i + index, new_contact);
                index++;
            }

//            std::cout << "ADDING index " << i << std::endl;
//            std::cout << "distance: " << distance << std::endl;
//            std::cout << "FOOTSTEP_SEQUENCE AT ITERATION " << i << std::endl;
//            _footstep_sequence[i].print();
//            std::cout << "NEXT_CONTACT: " << std::endl;
//            next_contact.print();

            Contact new_contact;
            new_contact = _footstep_sequence[i];
            new_contact.state.pose.translation().x() = (next_contact.state.pose.translation().x() + _footstep_sequence[i].state.pose.translation().x())/2;
            new_contact.state.pose.translation().y() = (next_contact.state.pose.translation().y() + _footstep_sequence[i].state.pose.translation().y())/2;
            new_contact.state.pose.linear().setIdentity();

//            std::cout << "NEW_CONTACT:" << std::endl;
//            new_contact.print();
//            std::cout << "inserting in position " << i+index << std::endl;

            _footstep_sequence.insert(_footstep_sequence.begin() + i + index, new_contact);

        }

        // remove footsteps when adjacent are below a threshold
//        if (distance < _step_size / 2)
//        {
//            std::cout << "REMOVING index " << i << std::endl;
//            std::cout << "distance: " << distance << std::endl;
//            _footstep_sequence[i].print();
//            next_contact.print();
//            _footstep_sequence.erase(_footstep_sequence.begin() + i, _footstep_sequence.begin() + i + _n_feet - 1);
//        }
    }
}

void DCMPlanner::clear()
{
    _footstep_sequence.clear();
    _cp_trj.clear();
    _com_trj.clear();
}

