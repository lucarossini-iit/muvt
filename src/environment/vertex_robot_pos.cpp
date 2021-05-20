#include <environment/vertex_robot_pos.h>

using namespace XBot::HyperGraph;
using namespace g2o;

VertexRobotPos::VertexRobotPos():
BaseDynamicVertex<Eigen::VectorXd>(),
_n_dof(0)
{
    setToOriginImpl();
    updateCache();
}

void VertexRobotPos::setNDoFs(int n)
{
    _n_dof = n;
    _estimate.resize(_n_dof);
    _estimate.setZero(_n_dof);
    updateCache();
}


void VertexRobotPos::setToOriginImpl() 
{
    _estimate.setZero(_n_dof);
    updateCache();
}

void VertexRobotPos::oplusImpl(const double* update) 
{
    std::cout << "INSIDE oplusImpl" << std::endl;
    Eigen::VectorXd q(_n_dof);
    q = _estimate;
    q = q + Eigen::VectorXd::Map(update, _n_dof);
    std::cout << "adding " << Eigen::VectorXd::Map(update, _n_dof).transpose() << " to " << _estimate.transpose() << std::endl;
    _estimate = q;
}

bool VertexRobotPos::setEstimateDataImpl(const number_t* est) 
{
    _estimate = Eigen::VectorXd::Map(est, _n_dof);
    return true;
}

bool VertexRobotPos::getEstimateData(number_t* est) const
{
    Eigen::VectorXd q = Eigen::VectorXd::Map(est, _n_dof);
    _estimate;
    return true;
}

int VertexRobotPos::estimateDimension() const 
{
    return _n_dof;
}

bool VertexRobotPos::setMinimalEstimateDataImpl(const number_t* est) 
{
    return setEstimateData(est);
}

bool VertexRobotPos::getMinimalEstimateData(number_t* est) const 
{
    return getEstimateData(est);
}

int VertexRobotPos::minimalEstimateDimension() const 
{
    return estimateDimension();
}

