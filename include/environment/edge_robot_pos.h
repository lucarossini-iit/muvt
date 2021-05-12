#ifndef EDGE_ROBOT_POS_H
#define EDGE_ROBOT_POS_H

#include <g2o/core/base_unary_edge.h>

#include <XBotInterface/ModelInterface.h>
#include <OpenSoT/utils/collision_utils.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>

#include <environment/vertex_robot_pos.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {
    
// The template argument defines the maximum number of link pairs of the robot and the number of DoFs of the robot
template <int M, int N>
class EdgeRobotPos : public BaseUnaryEdge<M, Eigen::VectorXd, VertexRobotPos<N>> {
public:
    typedef BaseUnaryEdge<M, Eigen::VectorXd, VertexRobotPos<N>> BaseEdge;
    EdgeRobotPos(XBot::ModelInterface::Ptr& model):
    _model(model),
    BaseUnaryEdge<M, Eigen::VectorXd, VertexRobotPos<N>>()
    { 
        auto urdf = _model->getUrdf();
        std::vector<urdf::LinkSharedPtr> links;
        urdf.getLinks(links);        
        for (auto link : links)
            _links.push_back(link->name); 
        
        urdf::ModelSharedPtr collision_urdf = boost::make_shared<urdf::Model>();
        if (collision_urdf->initParam("collision_urdf"))
            _dist = std::make_shared<ComputeLinksDistance>(*_model, collision_urdf);
        else
            ROS_ERROR("unable to find collision_urdf");
    }
    
    bool read(std::istream& is)
    {
//         Eigen::VectorXd p;
//         is >> p;
//         BaseEdge::setMeasurement(p);
        Eigen::VectorXd meas;
        meas.resize(M);
        internal::readVector(is, meas);
        BaseEdge::setMeasurement(meas);
        
        return is.good() || is.eof();
    }
    
    bool write(std::ostream& os) const
    {
        Eigen::VectorXd p = BaseEdge::measurement();
        os << p;

        return os.good();
    }
        
    void setObstacle(Eigen::Vector3d ob, int id)
    {
        _obstacle = ob;
        
        moveit_msgs::PlanningSceneWorld wc;
        moveit_msgs::CollisionObject coll;
        coll.header.frame_id = "world";
        coll.header.stamp = ros::Time::now();
        coll.id = "obstacle";
        
        coll.operation = moveit_msgs::CollisionObject::ADD;
        coll.primitives.resize(1);
        coll.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
        coll.primitives[0].dimensions = {0.2}; 
        
        geometry_msgs::Pose p;
        p.position.x = _obstacle(0);
        p.position.y = _obstacle(1);
        p.position.z = _obstacle(2);
        p.orientation.x = 0;
        p.orientation.y = 0;
        p.orientation.z = 0;
        p.orientation.w = 1;
        coll.primitive_poses = {p};
        
        wc.collision_objects.push_back(coll);
        _dist->setWorldCollisions(wc);
    }    
    
    void computeError()
    {
        const VertexRobotPos<N>* v1 = dynamic_cast<const VertexRobotPos<N>*>(BaseEdge::_vertices[0]);
        
        Eigen::VectorXd q = v1->estimate().q();
        _model->setJointPosition(q);
        _model->update();
        
        double eps = 0.1;
        double S = 0.05;
        double r = 0.2;
        int n = 2;

        auto distances = _dist->getLinkDistances();
        double distance = 0;    
        int index = 0;
        
        for (auto i : distances)
        {
//             if (i.getLinkNames().first == "obstacle" || i.getLinkNames().second == "obstacle")
//             {   
                distance += i.getDistance();
                if (-distance > -r - eps)
                    BaseEdge::_error(index) = 0;
                double value = pow((-distance-(-r-eps))/S, n);
                BaseEdge::_error(index) = value;
//             }
        }
    }
    
private:
    XBot::ModelInterface::Ptr _model;
    std::vector<std::string> _links;
    Eigen::Vector3d _obstacle;
    std::shared_ptr<ComputeLinksDistance> _dist;
}; } }

#endif