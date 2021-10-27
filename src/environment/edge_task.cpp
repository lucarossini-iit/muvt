#include <environment/edge_task.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeTask::EdgeTask():
BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos>()
{}

void EdgeTask::setReference(Eigen::VectorXd ref)
{
    _ref = ref;
}

void EdgeTask::resize()
{
    if (_ref.size() == 0)
        std::runtime_error("forgot to set the reference!");

    setDimension(_ref.size());
}

Eigen::VectorXd EdgeTask::getError() const
{
    return _error;
}

Eigen::VectorXd EdgeTask::getReference() const
{
    return _ref;
}

void EdgeTask::computeError()
{
    auto v = dynamic_cast<const VertexRobotPos*>(_vertices[0]);

    double S = 2;
    Eigen::VectorXd diff = v->estimate() - _ref;
//    std::cout << "v: " << v->estimate().transpose() << " ref: " << _ref.transpose() << "    = " << diff.transpose() << std::endl;
    for (int i = 0; i < v->estimate().size(); i++)
    {
        double value = sqrt(pow(20*diff(i), S*2));
        _error(i) = value;
    }
//    _error = 1000 * diff.cwiseProduct(diff);
}
