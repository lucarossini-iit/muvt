#include <muvt_core/environment/joint/edge_kinematic.h>

using namespace Muvt::HyperGraph;
using namespace g2o;

EdgeKinematic::EdgeKinematic(Muvt::ModelInterface::Ptr model):
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
    const VertexRobotPos* v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);
    _model->setJointPosition(v1->estimate());
    _model->update();
    if (_distal_link == "com")
    {
        _model->getPose("wheel_1", _ref1);
        _model->getPose("wheel_2", _ref2);
        _model->getPose("wheel_3", _ref3);
        _model->getPose("wheel_4", _ref4);
        _T_ref.translation() = (_ref1.translation() + _ref2.translation() + _ref3.translation() + _ref4.translation())/4;
    }
    else
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

    Eigen::Vector6d delta;
    Eigen::Vector3d delta_pos;
    Eigen::Vector3d p_com;

    const VertexRobotPos* v1 = dynamic_cast<const VertexRobotPos*>(_vertices[0]);
    _model->setJointPosition(v1->estimate());
    _model->update();

    if (_distal_link == "com")
    {
        Eigen::Affine3d T1, T2, T3, T4;
        _model->getPose("wheel_1", T1);
        _model->getPose("wheel_2", T2);
        _model->getPose("wheel_3", T3);
        _model->getPose("wheel_4", T4);
        _T_ref.translation() = (T1.translation() + T2.translation() + T3.translation() + T4.translation())/4;

        _model->getCOM(p_com);

        delta_pos = p_com - _T_ref.translation();
        delta << delta_pos(0), delta_pos(1), delta_pos(2), 0, 0, 0;
    }
    else
    {
        Eigen::Affine3d T;
        _model->getPose(_distal_link, "world", T);
        delta_pos = T.translation() - _T_ref.translation();

        // orientation error
        Eigen::Quaternion<double> quat(T.linear());
        Eigen::Quaternion<double> quat_ref(_T_ref.linear());
        Eigen::Vector3d delta_rot = quat.w()*Eigen::Vector3d(quat_ref.x(), quat_ref.y(), quat_ref.z()) - quat_ref.w()*Eigen::Vector3d(quat.x(), quat.y(), quat.z()) - Eigen::Vector3d(quat.x(), quat.y(), quat.z()).cross(Eigen::Vector3d(quat_ref.x(), quat_ref.y(), quat_ref.z()));

        delta << delta_pos(0), delta_pos(1), delta_pos(2), delta_rot(0), delta_rot(1), delta_rot(2);
//        if (delta(3) > 0.01)
//            std::cout << "distal_link: " << _distal_link << " delta: " << delta.transpose() << std::endl;
    }


    // convert in millimiters
//    delta_pos *= 1000;

    int ind = 0;
    for (auto index : _indices)
    {
//        std::cout << "index " << index << std::endl;
        _error(ind) = delta(index);
        ind++;
//        if (_distal_link == "com")
//            std::cout << "p_com: " << p_com.transpose() << "    ref: " << _T_ref.translation().transpose() << "   _error: " << _error.transpose() << " (delta: " << delta_pos.transpose() << ")" << std::endl;
    }
}
