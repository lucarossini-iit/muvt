#ifndef CONTACT_H
#define CONTACT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace XBot { namespace HyperGraph {

struct ContactState {
    Eigen::Affine3d pose;
}

class Contact {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Contact(std::string distal_link);

private:
    std::string _distal_link;

};
} }

#endif // CONTACT_H
