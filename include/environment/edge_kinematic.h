#ifndef EDGE_KINEMATIC_H
#define EDGE_KINEMATIC_H

#include <XBotInterface/ModelInterface.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>

#include <environment/unary_edge.h>
#include <environment/vertex_robot_pos.h>
#include <environment/obstacle.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {

class EdgeKinematic : public UnaryEdge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeKinematic(XBot::ModelInterface::Ptr model);

    bool read(std::istream& is)
    {
        return true;
    }

    bool write(std::ostream& os) const
    {
        auto v = dynamic_cast<VertexRobotPos*>(_vertices[0]);
        os << "vertex: " << v->estimate().transpose() << std::endl;
        os << "error: " << _error.transpose() << std::endl;

        return os.good();
    }

    bool setDistalLink(std::string distal_link);
    std::string getDistalLink() const {return _distal_link;}
    bool setIndices(std::vector<int> indices);
    void setReference(Eigen::Affine3d T_ref);
    void setBaseLink(std::string base_link);
    void computeError();
//    void linearizeOplus();


private:
    std::string _distal_link, _base_link;
    std::vector<int> _indices;
    Eigen::Affine3d _T_ref, _ref1, _ref2, _ref3, _ref4;

};
} }


#endif // EDGE_KINEMATIC_H
