#ifndef JOINT_LIMITS_H
#define JOINT_LIMITS_H

#include <g2o/core/base_unary_edge.h>

#include <environment/joint/vertex_robot_pos.h>

#include <XBotInterface/ModelInterface.h>

using namespace g2o;

namespace XBot { namespace HyperGraph {

class EdgeJointLimits : public BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos> {
public:
    EdgeJointLimits(XBot::ModelInterface::Ptr model);

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
    XBot::ModelInterface::Ptr _model;
};

} }

#endif
