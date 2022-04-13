#include <environment/joint/robot_pos.h>

using namespace XBot::HyperGraph;

RobotPos::RobotPos() 
{
    RobotPos(0);
}


RobotPos::RobotPos(int n):
_n(n)
{
    _q.resize(n);
    _q.setZero();
}

RobotPos::RobotPos(Eigen::VectorXd q):
_q(q)
{
    _n = _q.size();
}

RobotPos::RobotPos(Eigen::VectorXd q, int n) 
{
    if (n != q.size())
        std::runtime_error("'n' differs from 'q.size()'");
    
    _n = n;
    _q.resize(n);
    _q = q;
}

void RobotPos::setJointPosition(Eigen::VectorXd q) 
{
    _q = q;
}

void RobotPos::setNDoF(int n) 
{
    _n = n;
    _q.resize(n);
}



