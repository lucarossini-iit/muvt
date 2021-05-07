#ifndef EDGE_ROBOT_POS_H
#define EDGE_ROBOT_POS_H

#include <g2o/core/base_unary_edge.h>

#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/utils/collision_utils.h>

#include <environment/vertex_robot_pos.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {
    
// The template argument defines the number of DoFs of the robot
template <int N>
class EdgeRobotPos : public BaseUnaryEdge<N, Eigen::VectorXd, VertexRobotPos<N>> {
public:
    typedef BaseUnaryEdge<N, Eigen::VectorXd, VertexRobotPos<N>> BaseEdge;
    EdgeRobotPos(XBot::ModelInterface::Ptr& model):
    _model(model) 
    { 
        BaseUnaryEdge<N, Eigen::VectorXd, VertexRobotPos<N>>();
        auto urdf = _model->getUrdf();
        std::vector<urdf::LinkSharedPtr> links;
        urdf.getLinks(links);        
        for (auto link : links)
            _links.push_back(link->name); 
        
        _dist = std::make_shared<ComputeLinksDistance>(_model);
    }
    
    bool read(std::istream& is)
    {
        double p;
        is >> p;
        BaseEdge::setMeasurement(p);

        return true;
    }
    
    bool write(std::ostream& os) const
    {
        double p = BaseEdge::measurement();
        os << p;

        return os.good();
    }
    
    void setObstacle(Eigen::Vector3d ob)
    {
        _obstacle = ob;
    }    
    
    void computeError()
    {
        double eps = 0.1;
        double S = 0.05;
        double r = 0.2;
        int n = 2;
        BaseEdge::_error.clear();
        auto distances = _dist->getLinkDistances(r);
        double distance = 0;
        
        for (auto i : distances)
        {
            distance += i.getDistance();
            double value = pow((-distance-(-r-eps))/S, n);
            BaseEdge::_error.push_back(value);
        }
    }
    
private:
    XBot::ModelInterface::Ptr _model;
    std::vector<std::string> _links;
    Eigen::Vector3d _obstacle;
    std::shared_ptr<ComputeLinksDistance> _dist;
}; } }

#endif