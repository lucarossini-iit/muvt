#ifndef CONTACT_H
#define CONTACT_H

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Muvt { namespace HyperGraph {

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

inline Eigen::Vector3d compute3dError(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
  Eigen::Vector3d error;
  auto diff = v2 - v1;

  double S = 2, Sc = 100;

  error[0] = 1/exp(Sc*diff(0) - Sc*0.1) + exp(Sc*diff(0) - Sc*0.3);
  //if (v1->estimate().getDistalLink() == "l_sole") // FIXME hardcoded
      //error[1] = 1/exp(Sc*diff(1) + Sc*0.3) + exp(Sc*diff(1) + Sc*0.15);
  //else
      error[1] = 1/exp(Sc*diff(1) - Sc*0.15) + exp(Sc*diff(1) - Sc*0.3);
  error[2] = diff(2) * diff(2);
}

} }

#endif // CONTACT_H
