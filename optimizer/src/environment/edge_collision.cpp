#include <environment/edge_collision.h>
#include <visualization_msgs/MarkerArray.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeCollision::EdgeCollision(XBot::ModelInterface::Ptr model, std::shared_ptr<ComputeLinksDistance> cld, int max_pair_link):
UnaryEdge(model),
_cld(cld)
{
    resize(max_pair_link);
    ros::NodeHandle nh;
}

void EdgeCollision::resize(int size)
{
    setDimension(size);
}

void EdgeCollision::clear()
{
    _cld->removeAllWorldCollision();
}

void EdgeCollision::setObstacles(obstacles obs)
{
    _cld->removeAllWorldCollision();

//    if (obs.size() > 0)
//    {
        moveit_msgs::PlanningSceneWorld wc;
        for (auto obstacle : obs)
        {
            moveit_msgs::CollisionObject coll;
            coll.header.frame_id = obstacle.frame_id;
            coll.header.stamp = ros::Time::now();
            coll.id = "obstacle_" + std::to_string(obstacle.id);

            coll.operation = moveit_msgs::CollisionObject::ADD;
            coll.primitives.resize(1);
            coll.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
            coll.primitives[0].dimensions = {obstacle.size(0), obstacle.size(1), obstacle.size(2)};

//            coll.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
//            coll.primitives[0].dimensions = {obstacle.size(0)};

            geometry_msgs::Pose p;
            p.position.x = obstacle.position(0);    p.position.y = obstacle.position(1);    p.position.z = obstacle.position(2);
            tf::quaternionEigenToMsg(obstacle.orientation, p.orientation);
            coll.primitive_poses = {p};

            wc.collision_objects.push_back(coll);
        }

        _cld->setWorldCollisions(wc);
//    }
}

void EdgeCollision::computeError()
{
    int n_dof = _model->getJointNum();
    const VertexRobotPos* v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    _model->setJointPosition(v1->estimate());
    _model->update();

    double eps = 0.1;
    double S = 0.05;
    double r = 0.0;
    int n = 2;

    _error.setZero();

    auto dist = _cld->getLinkDistances(r+eps);
    int index = 0;

    for (auto i : dist)
    {
        double distance = 0;
        distance += i.getDistance();
//        if (i.getLinkNames().first.substr(0,14) == "world/obstacle" || i.getLinkNames().second.substr(0,14) == "world/obstacle")
//        {
            if (i.getLinkNames().first.substr(0,14) == "world/obstacle" || i.getLinkNames().second.substr(0,14) == "world/obstacle")
            {
                // use higher distance threshold to compensate inaccuracies of the camera
                eps = 0.1;
            }
            else
            {
                // use a lower distance threshold for self collision since link position is more precise
                eps = 0.02;
            }

            if (distance > r + eps)
                _error(index) = 0;
            else if (distance < 0 )
            {
                double value = -4*(r + eps)/S * distance;
                _error(index) = value;
            }
            else
            {
                double value = pow((-distance-(-r-eps))/S, n);
                _error(index) = value;
//                std::cout << i.getLinkNames().first << "  " << i.getLinkNames().second << "   " << i.getDistance() << std::endl;
            }
        index++;
//        }
    }
}

Eigen::VectorXd EdgeCollision::getError() const
{
    return _error;
}

