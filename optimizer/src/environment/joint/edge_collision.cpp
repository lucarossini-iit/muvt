#include <environment/joint/edge_collision.h>
#include <visualization_msgs/MarkerArray.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeCollision::EdgeCollision(XBot::ModelInterface::Ptr model, std::shared_ptr<ComputeLinksDistance> cld, int max_pair_link):
UnaryEdge(model),
_cld(cld),
_nh("")
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

void EdgeCollision::advertise()
{
    auto v = dynamic_cast<VertexRobotPos*>(_vertices[0]);
    _points_pub = _nh.advertise<visualization_msgs::MarkerArray>("points_id_" + std::to_string(v->id()), 1, true);
}

void EdgeCollision::setObstacles(const obstacles obs)
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
            if (obstacle.type == visualization_msgs::Marker::CUBE)
                coll.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
            else if (obstacle.type == visualization_msgs::Marker::CYLINDER)
                coll.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
            else if (obstacle.type == visualization_msgs::Marker::SPHERE)
                coll.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;

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
    VertexRobotPos* v1 = dynamic_cast<VertexRobotPos*>(_vertices[0]);

    _model->setJointPosition(v1->estimate());
    _model->update();

    double eps = 0.05;
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
        if (v1->id() == 47 && i.getLinkNames().first == "wheel_1" && i.getLinkNames().second.substr(0,14) == "world/obstacle")
        {
            std::cout << "id: " << v1->id() << "  " << i.getLinkNames().first << "  -  " << i.getLinkNames().second << ": " << distance << std::endl;
            auto frame1 = i.getClosestPoints().first;
            auto frame2 = i.getClosestPoints().second;

            visualization_msgs::MarkerArray ma;
            visualization_msgs::Marker point1;
            point1.action = visualization_msgs::Marker::ADD;
            point1.header.frame_id = "world";
            point1.header.stamp = ros::Time::now();
            point1.id = v1->id() + 1;
            point1.type = visualization_msgs::Marker::SPHERE;
            point1.scale.x = 0.02; point1.scale.y = 0.02; point1.scale.z = 0.02;
            point1.pose.position.x = frame1.p(0); point1.pose.position.y = frame1.p(1); point1.pose.position.z = frame1.p(2);
            point1.pose.orientation.x = 0; point1.pose.orientation.y = 0; point1.pose.orientation.z = 0; point1.pose.orientation.w = 1;
            point1.color.r = 1; point1.color.g = 1; point1.color.b = 1; point1.color.a = 1;

            visualization_msgs::Marker point2;
            point2.action = visualization_msgs::Marker::ADD;
            point2.header.frame_id = "world";
            point2.header.stamp = ros::Time::now();
            point2.id = v1->id() + 2;
            point2.type = visualization_msgs::Marker::SPHERE;
            point2.scale.x = 0.02; point2.scale.y = 0.02; point2.scale.z = 0.02;
            point2.pose.position.x = frame2.p(0); point2.pose.position.y = frame2.p(1); point2.pose.position.z = frame2.p(2);
            point2.pose.orientation.x = 0; point2.pose.orientation.y = 0; point2.pose.orientation.z = 0; point2.pose.orientation.w = 1;
            point2.color.r = 1; point2.color.g = 1; point2.color.b = 1; point2.color.a = 1;

            std::cout << "point1: " << point1.pose.position.x << ", " << point1.pose.position.y << ", " << point1.pose.position.z << std::endl;
            std::cout << "point2: " << point2.pose.position.x << ", " << point2.pose.position.y << ", " << point2.pose.position.z << std::endl;

            ma.markers.push_back(point1);
            ma.markers.push_back(point2);

            _points_pub.publish(ma);
        }

        if (i.getLinkNames().first.substr(0,14) == "world/obstacle" || i.getLinkNames().second.substr(0,14) == "world/obstacle")
        {
            // use higher distance threshold to compensate inaccuracies of the camera
            eps = 0.05;
        }
        else
        {
            // use a lower distance threshold for self collision since link position is more precise
            eps = 0.02;
        }

        if (distance > r + eps)
            _error(index) = 0;
        else
        {
            // TODO: investigate random perturbation of the vertex when it get stuck in the obstacle (maybe should be done in optimizer.cpp)
//            if (distance < 0)
//            {
//                Eigen::VectorXd q_rand = v1->estimate();
//                q_rand.setRandom();
//                VertexRobotPos* v_rand = new VertexRobotPos();
//                v_rand->setDimension(_model->getJointNum());
//                v_rand->setEstimate(q_rand);
//                v_rand->setId(v1->id());
//                _vertices[0] = v_rand;
//            }
            double value = pow((-distance-(-r-eps))/S, n);
//            double value = -distance-eps;
            _error(index) = value;
//            std::cout << i.getLinkNames().first << "  " << i.getLinkNames().second << "   " << i.getDistance() << std::endl;
        }
        index++;
    }
}

Eigen::VectorXd EdgeCollision::getError() const
{
    return _error;
}

