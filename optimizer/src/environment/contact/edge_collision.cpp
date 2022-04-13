#include <environment/contact/edge_collision.h>

using namespace g2o;
using namespace XBot::HyperGraph;

EdgeCollision::EdgeCollision():
BaseUnaryEdge<3, Eigen::Vector3d, VertexContact>()
{}

void EdgeCollision::setObstacles(const Eigen::Vector3d obs)
{
    _obstacle = obs;
}

void EdgeCollision::computeError()
{
    auto v = dynamic_cast<VertexContact*>(_vertices[0]);

    Eigen::Vector3d diff = v->estimate().state.pose.translation() - _obstacle;

    double r = 0.2;
    double eps = 0.0;
    double S = 0.01;
    int n = 2;

    for (int i = 0; i < diff.size(); i++)
    {
        if (diff(i) > r + eps)
            _error(i) = 0;
        else
        {
            double value = pow((-diff(i)-(-r-eps))/S, n);
            _error(i) = value;
        }
    }
}
