#include <environment/edge_collision.h>
#include <visualization_msgs/MarkerArray.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeCollision::EdgeCollision(XBot::ModelInterface::Ptr& model, std::shared_ptr<ComputeLinksDistance> cld, int max_pair_link):
BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos>(),
_model(model),
_cld(cld)
{
    resize(max_pair_link);
    ros::NodeHandle nh;

    _point1_pub = nh.advertise<visualization_msgs::Marker>("point1", 1, true);
    _point2_pub = nh.advertise<visualization_msgs::Marker>("point2", 1, true);
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

    auto distance_check = _cld->getLinkDistances();
    for (auto distance : distance_check)
    {
        if (distance.getLinkNames().first == "arm1_7" && distance.getLinkNames().second.substr(0,14) == "world/obstacle")
        {
            std::cout << "link0: " << distance.getLinkNames().first << "    link1: " << distance.getLinkNames().second << std::endl;
            std::cout << "point0: " << distance.getClosestPoints().first.p << "     point1: " << distance.getClosestPoints().second.p << std::endl;
            std::cout << "distance: " << distance.getDistance() << std::endl;

            visualization_msgs::Marker m1, m2;
            m1.header.frame_id = "D435i_head_camera_color_optical_frame";
            m1.header.stamp = ros::Time::now();
            m1.header.seq = 0;
            m1.type = visualization_msgs::Marker::CUBE;
            m1.pose.position.x = distance.getClosestPoints().first.p(0);
            m1.pose.position.y = distance.getClosestPoints().first.p(1);
            m1.pose.position.z = distance.getClosestPoints().first.p(2);
            m1.pose.orientation.x = 0; m1.pose.orientation.y = 0; m1.pose.orientation.z = 0; m1.pose.orientation.w = 1;
            m1.color.r = 0; m1.color.g = 0; m1.color.b = 1; m1.color.a = 1;
            m1.action = visualization_msgs::Marker::ADD;
            m1.scale.x = 0.05; m1.scale.y = 0.05; m1.scale.z = 0.05;
            _point1_pub.publish(m1);

            m2.header.frame_id = "D435i_head_camera_color_optical_frame";
            m2.header.stamp = ros::Time::now();
            m2.header.seq = 1;
            m2.type = visualization_msgs::Marker::CUBE;
            m2.pose.position.x = distance.getClosestPoints().second.p(0);
            m2.pose.position.y = distance.getClosestPoints().second.p(1);
            m2.pose.position.z = distance.getClosestPoints().second.p(2);
            m2.pose.orientation.x = 0; m2.pose.orientation.y = 0; m2.pose.orientation.z = 0; m2.pose.orientation.w = 1;
            m2.color.r = 0; m2.color.g = 0; m2.color.b = 1; m2.color.a = 1;
            m2.action = visualization_msgs::Marker::ADD;
            m2.scale.x = 0.05; m2.scale.y = 0.05; m2.scale.z = 0.05;
            _point2_pub.publish(m2);

        }
    }
    auto dist = _cld->getLinkDistances(r);
    auto end = std::next(dist.begin(), _error.size());
    std::list<LinkPairDistance> distances(dist.begin(), end);
    int index = 0;

    for (auto i : distances)
    {
        if (i.getLinkNames().first.substr(0,14) == "world/obstacle" || i.getLinkNames().second.substr(0,14) == "world/obstacle")
        {
//            std::cout << "link0: " << i.getLinkNames().first << "   link2: " << i.getLinkNames().second << "    distance: " << i.getDistance() << std::endl;
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

