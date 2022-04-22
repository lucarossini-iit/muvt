#ifndef EDGE_COLLISION_H
#define EDGE_COLLISION_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <g2o/core/base_unary_edge.h>

#include <muvt/environment/contact/vertex_contact.h>

namespace Muvt { namespace HyperGraph {

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

    void computeError();

private:
    Eigen::Vector3d _obstacle;

};
} }

#endif // EDGE_COLLISION_H
