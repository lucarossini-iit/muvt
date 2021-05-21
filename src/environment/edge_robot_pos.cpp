#include <environment/edge_robot_pos.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeRobotPos::EdgeRobotPos(XBot::ModelInterface::Ptr& model, int max_pair_link):
BaseUnaryEdge<int(10), Eigen::VectorXd, VertexRobotPos>(),
_model(model),
_max_pair_link(max_pair_link)
{ 
    urdf::ModelSharedPtr collision_urdf = boost::make_shared<urdf::Model>();
    if (collision_urdf->initParam("collision_urdf"))
    {
        std::cout << "creating ComputeLinkDistance..." << std::endl;
        _dist = std::make_shared<ComputeLinksDistance>(*_model, collision_urdf);
        std::cout << "ComputeLinksDistance done!" << std::endl;
    }
    else
        ROS_ERROR("unable to find collision_urdf");
    
//     _error.resize(_max_pair_link);
}

void EdgeRobotPos::setObstacle(Eigen::Vector3d ob, int id)
{
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
    p.position.x = ob(0);
    p.position.y = ob(1);
    p.position.z = ob(2);
    p.orientation.x = 0;
    p.orientation.y = 0;
    p.orientation.z = 0;
    p.orientation.w = 1;
    coll.primitive_poses = {p};
    
    wc.collision_objects.push_back(coll);
    _dist->setWorldCollisions(wc);
}

void EdgeRobotPos::computeError() 
{
    int n_dof = _model->getJointNum();
    const VertexRobotPos* v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);
    
    Eigen::VectorXd q = v1->estimate();
    _model->setJointPosition(q);
    _model->update();
    
    double eps = 0.01;
    double S = 0.05;
    double r = 0.2;
    int n = 2;
    
    _error.setZero();

    auto distances = _dist->getLinkDistances();
    int index = 0;
    
    for (auto i : distances)
    {
        if (i.getLinkNames().first == "world/obstacle" || i.getLinkNames().second == "world/obstacle")
        {   
            double distance = 0;
            distance += i.getDistance();
            if (distance > r + eps)
                _error(index) = 0;
            else
            {
                double value = pow((-distance-(-r-eps))/S, n);
                _error(index) = value;
            }
        }
    index++;
    }
    
    std::cout << "ERROR: " << _error.transpose() << std::endl;
}

Eigen::VectorXd EdgeRobotPos::getError() const
{
    return _error;
}



