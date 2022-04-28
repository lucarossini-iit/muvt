#ifndef MUVT_CORE_EDGE_COLLISION_H
#define MUVT_CORE_EDGE_COLLISION_H

#include <MuvtInterface/ModelInterface.h>
#include <OpenSoT/utils/collision_utils.h>
#include <OpenSoT/constraints/velocity/CollisionAvoidance.h>

#include <eigen_conversions/eigen_msg.h>

#include <environment/joint/unary_edge.h>
#include <environment/joint/vertex_robot_pos.h>
#include <environment/obstacle.h>
#include <octomap_msgs/OctomapWithPose.h>

using namespace g2o;

namespace Muvt { namespace HyperGraph {

class EdgeCollision : public UnaryEdge {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::vector<obstacle> obstacles;

    EdgeCollision(Muvt::ModelInterface::Ptr model,
                  std::shared_ptr<ComputeLinksDistance> dist,
                  int max_pair_link);

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

    void setObstacles(const obstacles obs);
    void resize(int size);
    void clear();
    void computeError();
    void advertise();

    Eigen::VectorXd getError() const;

    unsigned int ID;

private:
    std::shared_ptr<ComputeLinksDistance> _cld;
    ros::Publisher _points_pub;
    ros::NodeHandle _nh;

}; } }

#endif // MUVT_CORE_EDGE_COLLISION_H
