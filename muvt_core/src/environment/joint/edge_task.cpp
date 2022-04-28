#include <muvt_core/environment/joint/edge_task.h>

using namespace Muvt::HyperGraph;
using namespace g2o;

EdgeTask::EdgeTask(Muvt::ModelInterface::Ptr model):
UnaryEdge(model)
{}

void EdgeTask::setReference(Eigen::VectorXd ref)
{
    _ref = ref;
    _model->setJointPosition(_ref);
    _model->update();

    if (_ee.size() == 0)
    {
        std::runtime_error("forgot to specify end effectors!");
    }

    for (int i = 0; i < _ee.size(); i++)
    {
        _model->getPose(_ee[i], _T_ref[i]);
    }
}

bool EdgeTask::resize()
{
    if (_ref.size() == 0)
    {
        std::runtime_error("forgot to set the reference!");
        return false;
    }

    if (_ee.size() == 0)
    {
        std::runtime_error("forgot to specify end effectors!");
        return false;
    }

//    setDimension(_ref.size());
    setDimension(_ee.size()*3);

    return true;
}

void EdgeTask::setEndEffectors(std::vector<std::string> ee)
{
    _ee = ee;
    _T_ref.resize(_ee.size());
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

    Eigen::Affine3d T;
    _model->setJointPosition(v->estimate());
    _model->update();

    std::vector<double> diff_vect;

    for (int i = 0; i < _ee.size(); i++)
    {
        _model->getPose(_ee.at(i), T);
        diff_vect.push_back(T.translation().x() - _T_ref[i].translation().x());
        diff_vect.push_back(T.translation().y() - _T_ref[i].translation().y());
        diff_vect.push_back(T.translation().z() - _T_ref[i].translation().z());
    }

    Eigen::VectorXd diff = Eigen::VectorXd::Map(diff_vect.data(), diff_vect.size());

    _error = diff;
}
