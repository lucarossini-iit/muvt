    #include <environment/edge_collision.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeCollision::EdgeCollision(XBot::ModelInterface::Ptr& model, std::shared_ptr<ComputeLinksDistance> cld, int max_pair_link):
BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos>(),
_model(model),
_cld(cld)
{
    resize(max_pair_link);
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

        geometry_msgs::Pose p;
        p.position.x = obstacle.position(0);    p.position.y = obstacle.position(1);    p.position.z = obstacle.position(2);
        tf::quaternionEigenToMsg(obstacle.orientation, p.orientation);
        coll.primitive_poses = {p};

        wc.collision_objects.push_back(coll);
    }

    _cld->setWorldCollisions(wc);
}

void EdgeCollision::computeError()
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

    _error.setZero();

    auto dist = _cld->getLinkDistances(r);
    auto end = std::next(dist.begin(), _error.size());
    std::list<LinkPairDistance> distances(dist.begin(), end);
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

Eigen::VectorXd EdgeCollision::getError() const
{
    return _error;
}

