#ifndef CONTACT_H
#define CONTACT_H

#include <iostream>

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

    // time
    double time;

    bool active;
};

class Contact {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Contact(std::string distal_link = "");

    Contact operator=(const Contact& other);

    void setDistalLink(std::string distal_link);
    std::string getDistalLink() const;

    // operator to combine two RobotPos (sum of vectors)
    inline Contact operator + (const Contact& c) const
    {
        Contact result(*this);
        result.state.pose.translation() += c.state.pose.translation();
        result.state.pose.linear() *= c.state.pose.linear();
        return result;
    }

    inline Contact operator += (const Contact& c)
    {
        state = c.state;
        setDistalLink(c.getDistalLink());
        return *this;
    }

    void print();

    ContactState state;

private:
    std::string _distal_link;

};
} }

#endif // CONTACT_H
