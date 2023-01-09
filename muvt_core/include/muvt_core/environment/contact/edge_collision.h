#ifndef MUVT_CORE_EDGE_COLLISION_H
#define MUVT_CORE_EDGE_COLLISION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <g2o/core/base_unary_edge.h>

#include <muvt_core/environment/contact/vertex_contact.h>

namespace Muvt { namespace HyperGraph { namespace ContactSpace {

class EdgeCollision : public BaseUnaryEdge<1, double, VertexContact> {
public:
    EdgeCollision();

    bool read(std::istream& is)
    {
        return true;
    }

    bool write(std::ostream& os) const
    {
        return os.good();
    }

    void setObstacles(const Eigen::Vector3d obs);

    Eigen::Vector3d getObstacle() const { return _obstacle; }

    void computeError();

private:
    Eigen::Vector3d _obstacle;

};
} } }

#endif // MUVT_CORE_EDGE_COLLISION_H
