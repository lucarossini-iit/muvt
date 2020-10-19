#include <g2o/core/base_vertex.h>
#include <g2o/config.h>
#include "g2o/core/hyper_graph_action.h"

#include <environment/se3.h>

namespace XBot { namespace Vertex {
    
    class Vertex_SE3 : public g2o::BaseVertex<7, XBot::Vertex::SE3> {
    public:
        EIGEN_ALIGNED_OPERATOR_NEW;
        
        Vertex_SE3();
        
        virtual void setToOriginImpl()
        {
            _estimate = SE3();
        };
        
        virtual void oplusImpl (double* update)
        {
            SE3 up(Eigen::Vector3d(v[0], v[1], v[2]), Eigen::Quaternion<double>(v[3], v[4], v[5], v[6]));
            
            _estimate = _estimate * up;
        }
        
        virtual bool read (std::istream& is);
        virtual bool write (std::ostream& os) const;
        
    private:
        SE3 _estimate;
        
    }; } }
