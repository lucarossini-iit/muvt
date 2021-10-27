#include <environment/edge_robot_pos.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeRobotPos::EdgeRobotPos(XBot::ModelInterface::Ptr& model, int max_pair_link):
BaseUnaryEdge<int(30), Eigen::VectorXd, VertexRobotPos>(),
_model(model),
_max_pair_link(max_pair_link)
{ 
    urdf::ModelSharedPtr collision_urdf = std::make_shared<urdf::Model>();
    if (collision_urdf->initParam("collision_urdf"))
    {
        std::cout << "creating ComputeLinkDistance..." << std::endl;
        _dist = std::make_shared<ComputeLinksDistance>(*_model, collision_urdf);
        std::cout << "ComputeLinksDistance done!" << std::endl;
    }
    else
        ROS_ERROR("unable to find collision_urdf");
    
}

void EdgeRobotPos::addObstacle(Eigen::Vector3d ob, int id)
{
    moveit_msgs::PlanningSceneWorld wc;
    moveit_msgs::CollisionObject coll;
    coll.header.frame_id = "world";
    coll.header.stamp = ros::Time::now();
    coll.id = "obstacle" + std::to_string(id);
    
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

void EdgeRobotPos::updateObstacle(Eigen::Vector3d ob, int ind)
{
    Eigen::Affine3d T;
    KDL::Frame Tk;
    T.translation() = ob;
    T.linear() = Eigen::Matrix3d::Identity();
    tf::transformEigenToKDL(T, Tk);
    _dist->moveWorldCollision("obstacle" + std::to_string(ind), Tk);
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
    double r = 0.1;
    int n = 2;
    
    _error.setZero(_max_pair_link);

    auto distances = _dist->getLinkDistances();
    int index = 0;
    
    for (auto i : distances)
    {
        if (i.getLinkNames().first.substr(0,14) == "world/obstacle" || i.getLinkNames().second.substr(0,14) == "world/obstacle")
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
}

Eigen::VectorXd EdgeRobotPos::getError() const
{
    return _error;
}



