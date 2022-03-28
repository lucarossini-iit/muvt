#ifndef CONTACT_H
#define CONTACT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace XBot { namespace HyperGraph {

struct ContactState {
    Eigen::Affine3d pose;

    // ZMP
    Eigen::Vector3d zmp;
    // CP
    Eigen::Vector3d cp;
};

class Contact {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Contact(std::string distal_link);

    Contact operator=(const Contact& other);

    void setDistalLink(std::string distal_link);
    std::string getDistalLink() const;

    ContactState state;

private:
    std::string _distal_link;

};
} }

#endif // CONTACT_H
