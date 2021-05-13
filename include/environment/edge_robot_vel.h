#ifndef EDGE_ROBOT_VEL_H
#define EDGE_ROBOT_VEL_H

#include <g2o/core/base_binary_edge.h>

#include <XBotInterface/ModelInterface.h>

#include <environment/vertex_robot_pos.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {

// The template argument defines the maximum number of link pairs of the robot and the number of DoFs of the robot
template <int N>
class EdgeRobotVel : public BaseBinaryEdge<N, Eigen::VectorXd, VertexRobotPos<N>, VertexRobotPos<N>> {
public:
    typedef BaseBinaryEdge<N, Eigen::VectorXd, VertexRobotPos<N>, VertexRobotPos<N>> BaseEdge;
    
    EdgeRobotVel(XBot::ModelInterface::Ptr model) : 
    BaseBinaryEdge<N, Eigen::VectorXd, VertexRobotPos<N>, VertexRobotPos<N>>(),
    _model(model)
    {
        _model->getVelocityLimits(_vel_max);
        _vel_max = _vel_max / 100;
        _vel_min = -_vel_max;
    };
    
    bool read(std::istream& is)
    {}
    
    bool write(std::ostream& os) const
    {}
    
    void computeError()
    {
        auto v1 = dynamic_cast<const VertexRobotPos<N>*>(BaseEdge::_vertices[0]);
        auto v2 = dynamic_cast<const VertexRobotPos<N>*>(BaseEdge::_vertices[1]);
        
        Eigen::VectorXd q1(N), q2(N), diff(N), min(N), max(N);
        q1 = v1->estimate().q();
        q2 = v2->estimate().q();
        diff = q2 - q1;
        
        for (int i = 0; i < diff.size(); i++)
        {
            if (diff(i) > _vel_max(i) || diff(i) < _vel_min(i))
                BaseEdge::_error(i) = 100;
            else
                BaseEdge::_error(i) = 0;
        }
    }
    
private:
    XBot::ModelInterface::Ptr _model;
    
    Eigen::VectorXd _vel_min, _vel_max;
    
    
    
}; } }

#endif