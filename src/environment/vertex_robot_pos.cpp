#include <environment/vertex_robot_pos.h>

using namespace XBot::HyperGraph;
using namespace g2o;

VertexRobotPos::VertexRobotPos():
BaseDynamicVertex<Eigen::VectorXd>()
{}

bool VertexRobotPos::setDimensionImpl(int newDimension)
{
    int oldDimension = dimension();

    // Handle the special case this is the first time
    if (oldDimension == Eigen::Dynamic) {
      _estimate.resize(newDimension);
      _estimate.setZero();
      return true;
    }

    _estimate.conservativeResize(newDimension);

    // If the state has expanded, pad with zeros
    if (oldDimension < newDimension)
      _estimate.tail(newDimension-oldDimension).setZero();

    return true;
}


void VertexRobotPos::setToOriginImpl() 
{
    _estimate.setZero(_dimension);
}

void VertexRobotPos::oplusImpl(const double* update) 
{
    Eigen::VectorXd q(_dimension);
    q = _estimate;
    q = q + Eigen::VectorXd::Map(update, _n_dof);
    std::cout << "adding " << Eigen::VectorXd::Map(update, _dimension).transpose() << " to " << _estimate.transpose() << std::endl;
    _estimate = q;
}

bool VertexRobotPos::setEstimateDataImpl(const number_t* est) 
{
    _estimate = Eigen::VectorXd::Map(est, _dimension);
    return true;
}

bool VertexRobotPos::getEstimateData(number_t* est) const
{
    Eigen::VectorXd q = Eigen::VectorXd::Map(est, _dimension);
    _estimate;
    return true;
}

int VertexRobotPos::estimateDimension() const 
{
    return _dimension;
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

