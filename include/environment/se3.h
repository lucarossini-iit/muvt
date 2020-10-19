#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include "g2o/core/hyper_graph_action.h"

namespace XBot { namespace Vertex {
    
    class SE3 {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        SE3() : _t(0, 0, 0), _q(Eigen::Quaternion<double>::Identity()) {};  
        SE3(Eigen::Affine3d pose) : _t(pose.translation()), _q(Eigen::Quaternion<double>(pose.linear())) {}
        SE3(Eigen::Vector3d t, Eigen::Matrix3d R) : _t(t), _q(Eigen::Quaternion<double>(R)) {};
        SE3(Eigen::Vector3d t, Eigen::Quaternion<double> q) : _t(t), _q(q) {};
        
        // Copy Constructor
        SE3(const SE3& obj);
        
        // Copy Assignment
        SE3 operator= (const SE3& obj);
        
        // Used to propagate the state
        SE3 operator* (const SE3& obj) const;
                
        // Setter
        void setRotation(Eigen::Quaternion<double> q) {_q = q;};
        void setTranslation(Eigen::Vector3d t) {_t = t;};
        
        // Getter
        Eigen::Vector3d translation() const;
        Eigen::Quaternion<double> rotation() const;
        
        void fromVector(const Eigen::VectorXd& v);
        
        // Element in the Eigen::Vector are stored as (x,y,z  x,y,z,w)
        Eigen::VectorXd toVector() const;
        
    private:
        Eigen::Quaternion<double> _q;
        Eigen::Vector3d _t;
    };
} }