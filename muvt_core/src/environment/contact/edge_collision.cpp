#include <muvt_core/environment/contact/edge_collision.h>

using namespace g2o;
using namespace Muvt::HyperGraph;

EdgeCollision::EdgeCollision():
BaseUnaryEdge<1, double, VertexContact>()
{}

void EdgeCollision::setObstacles(const Eigen::Vector3d obs)
{
    _obstacle = obs;
}

void EdgeCollision::computeError()
{
    auto v = dynamic_cast<VertexContact*>(_vertices[0]);

    Eigen::Vector3d diff = v->estimate().state.pose.translation() - _obstacle;
    double diff_norm = diff.norm();

    double r = 0.3;
    double eps = 0.1;
    double S = 0.01;
    int n = 2;
    bool res = false;

    if (diff_norm > r + eps)
         _error[0] =  0;
    else
    {
        double value = pow((-diff_norm-(-r-eps))/S, n);
        _error[0] = value;
        res = true;
    }
}

