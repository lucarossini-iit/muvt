#include <environment/edge_kinematic.h>

using namespace XBot::HyperGraph;
using namespace g2o;

EdgeKinematic::EdgeKinematic(XBot::ModelInterface::Ptr model):
UnaryEdge(model)
{
//    setDimension(3);
}

bool EdgeKinematic::setDistalLink(std::string distal_link)
{
//    Eigen::Affine3d T;
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

//void EdgeKinematic::linearizeOplus()
//{
//    Eigen::MatrixXd J;
//    const VertexRobotPos* v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);
//    _model->setJointPosition(v1->estimate());
//    _model->update();
//    _model->getJacobian(_distal_link, J);
//    _jacobianOplusXi = J.row(_indices[0]);
//}

void EdgeKinematic::computeError()
{
    _error.setZero();

    const VertexRobotPos* v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);
    _model->setJointPosition(v1->estimate());
    _model->update();

    Eigen::Affine3d T;
    _model->getPose(_distal_link, "world", T);
    Eigen::Vector3d delta_pos = T.translation() - _T_ref.translation();
    // convert in millimiters
    delta_pos *= 1000;
//    if (delta_pos(2) > 1)
//        std::cout << "id: " << v1->id() << "  distal_link: " << _distal_link << "  -  pos:" << T.translation().transpose() << "   ref: " << _T_ref.translation().transpose() << "   delta: " << delta_pos.transpose() << std::endl;
    for (int i = 0; i < delta_pos.size(); i++)
        delta_pos(i) = pow(delta_pos(i), 2);

//    delta_pos.lpNorm<1>();

    int ind = 0;
    for (auto index : _indices)
    {
        _error(ind) = delta_pos(index);
        ind++;
    }

//    std::cout << _error.transpose()  << std::endl;

//    std::cout << "_error: " << _error.transpose() << std::endl;

//    if(delta_pos(2) > 0.1)
//    {
//        std::cout << "id: " << v1->id() << "   ref: " << _T_ref.translation().z() << "  current: " << T.translation().z() << "  error: " << delta_pos(2) << std::endl;
//    }
}
