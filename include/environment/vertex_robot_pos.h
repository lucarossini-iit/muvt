#ifndef VERTEX_ROBOT_POS_H
#define VERTEX_ROBOT_POS_H

#include <g2o/core/base_vertex.h>

#include <environment/robot_pos.h>

#include <iostream>

using namespace g2o;

namespace XBot { namespace HyperGraph {

// The template argument defines the number of DoFs of the robot
template <int N>
class VertexRobotPos : public BaseVertex<N, RobotPos>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexRobotPos() { BaseVertex<N, RobotPos>(); }
    
    virtual void setToOriginImpl() { BaseVertex<N,RobotPos>::_estimate = RobotPos(N); }
    
    virtual void oplusImpl(const double* update)
    {
        Eigen::VectorXd q(N);
        q = BaseVertex<N,RobotPos>::_estimate.q();
        q = q + Eigen::Map<const Eigen::VectorXd>(update);
        BaseVertex<N,RobotPos>::_estimate = RobotPos(q, N);
    }
    
    virtual bool setEstimateDataImpl(const number_t* est)
    {
        BaseVertex<N,RobotPos>::_estimate=RobotPos(Eigen::Map<const Eigen::VectorXd>(est));
        return true;
    }
    
    virtual bool getEstimateData(number_t* est) const
    {
        Eigen::Map<Eigen::VectorXd> q(est);
        q = BaseVertex<N, RobotPos>::_estimate.q();
        return true;
    }
    
    virtual int estimateDimension() const { return N; }
    
    virtual bool setMinimalEstimateDataImpl(const number_t* est)
    {
        return BaseVertex<N, RobotPos>::setEstimateData(est);
    }

    virtual bool getMinimalEstimateData(number_t* est) const 
    {
        return getEstimateData(est);
    }
    
    virtual int minimalEstimateDimension() const { return N; }
    
    virtual bool read(std::istream& is) 
    {
        Eigen::VectorXd p;
        bool state = internal::readVector(is, p);
        BaseVertex<N, RobotPos>::setEstimate(p);
        return state;
    }
    
    virtual bool write(std::ostream& os)
    {
        return internal::writeVector(os, BaseVertex<N, RobotPos>::estimate().q());
    }
    
};
    
} }


#endif