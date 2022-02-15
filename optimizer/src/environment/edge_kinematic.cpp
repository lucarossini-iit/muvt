#include <environment/edge_kinematic.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeKinematic::EdgeKinematic(XBot::ModelInterface::Ptr model):
UnaryEdge(model)
{}

bool EdgeKinematic::setDistalLink(std::string distal_link)
{
    Eigen::Affine3d T;
//    if (!_model->getPose(distal_link, T));
//    {
//        throw std::runtime_error("setting a non existing link as distal link!");
//        return false;
//    }

    _distal_link = distal_link;
    return true;
}

bool EdgeKinematic::setIndices(std::vector<int> indices)
{
    if (std::any_of(indices.begin(), indices.end(), [](int i){ return i < 0 || i > 5; }))
    {
        throw std::runtime_error("indices must be in the interval 0 -> 5");
        return false;
    }

    _indices = indices;

    // resize _error vector
    setDimension(_indices.size());

    return true;
}

void EdgeKinematic::setReference(Eigen::Affine3d T_ref)
{
    _T_ref = T_ref;
}

void EdgeKinematic::computeError()
{
    _error.setZero();

    const VertexRobotPos* v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);
    _model->setJointPosition(v1->estimate());
    _model->update();

    Eigen::Affine3d T;
    _model->getPose(_distal_link, T);
    Eigen::Vector3d delta_pos = T.translation() - _T_ref.translation();
    for (int i = 0; i < delta_pos.size(); i++)
        delta_pos(i) = pow(delta_pos(i) * 100, 2);

    int ind = 0;
    for (auto index : _indices)
    {
        _error(ind) = delta_pos(index);
        ind++;
    }

//    for (int i = 0; i < _error.size(); i++)
//    {
//        if(_error(i) > 0.1)
//        {
//            std::cout << "id: " << v1->id() << "   ref: " << _T_ref.translation().z() << "  current: " << T.translation().z() << "  error: " << _error(i) << std::endl;
//            break;
//        }

//    }

}
