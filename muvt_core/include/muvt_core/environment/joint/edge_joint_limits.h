#ifndef MUVT_CORE_JOINT_LIMITS_H
#define MUVT_CORE_JOINT_LIMITS_H

#include <g2o/core/base_unary_edge.h>

#include <environment/joint/vertex_robot_pos.h>

#include <MuvtInterface/ModelInterface.h>

using namespace g2o;

namespace Muvt { namespace HyperGraph {

class EdgeJointLimits : public BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos> {
public:
    EdgeJointLimits(Muvt::ModelInterface::Ptr model);

    bool read(std::istream& is)
    {
        Eigen::VectorXd meas;
        internal::readVector(is, meas);
        setMeasurement(meas);

        return is.good() || is.eof();
    }

    bool write(std::ostream& os) const
    {
        Eigen::VectorXd p = measurement();
        os << p;

        return os.good();
    }

    void resize();

    void computeError();

    Eigen::VectorXd getError() const;

private:
    Muvt::ModelInterface::Ptr _model;
};

} }

#endif
