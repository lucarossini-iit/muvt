#ifndef EDGE_ROBOT_POS_H
#define EDGE_ROBOT_POS_H

#include <g2o/core/base_unary_edge.h>

#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/utils/collision_utils.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>

#include <environment/vertex_robot_pos.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {
    
class EdgeRobotPos : public BaseUnaryEdge<10, Eigen::VectorXd, VertexRobotPos> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    EdgeRobotPos(XBot::ModelInterface::Ptr& model,
                 int max_pair_link);
    
    bool read(std::istream& is)
    {
        Eigen::VectorXd meas;
//         meas.resize(_max_pair_link);
        internal::readVector(is, meas);
        setMeasurement(meas);
        
        return is.good() || is.eof();
    }
    
    bool write(std::ostream& os) const
    {
        Eigen::VectorXd p = measurement();
        os << p;

        return os.good();
    }
        
    void setObstacle(Eigen::Vector3d ob, int id);    
    
    void computeError();
    
    Eigen::VectorXd getError() const;
    
private:
    XBot::ModelInterface::Ptr _model;
    int _max_pair_link;
    std::shared_ptr<ComputeLinksDistance> _dist;
}; } }

#endif