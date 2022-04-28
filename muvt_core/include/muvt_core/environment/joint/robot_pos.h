#ifndef MUVT_CORE_ROBOT_POS_H
#define MUVT_CORE_ROBOT_POS_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Muvt { namespace HyperGraph {

class RobotPos {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    RobotPos();
    RobotPos(int n);
    RobotPos(Eigen::VectorXd q);
    RobotPos(Eigen::VectorXd q, int n);
    
    inline const Eigen::VectorXd q() const { return _q; }
    void setJointPosition(Eigen::VectorXd q);
    
    inline const int n_dof() const { return _n; }
    void setNDoF(int n);
    
    // operator to combine two RobotPos (sum of vectors)
    inline RobotPos operator + (const RobotPos& q) const
    {
        RobotPos result(*this);
        result += q;
        return result;
    }
    
    inline RobotPos operator += (const RobotPos& q) 
    {
        _q += q._q;
        return *this;
    }
    
    inline double operator [] (const int i)
    {
        if (i > _n - 1)
            std::runtime_error("you are requesting an elemen outside the 'q' size!");
        return _q(i);
    }
        
protected:
    Eigen::VectorXd _q;
    int _n;
};
    
} }

#endif
