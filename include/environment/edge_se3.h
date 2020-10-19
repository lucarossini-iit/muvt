#include <g2o/config.h>
#include <g2o/core/base_binary_edge.h>

#include <environment/se3.h>
#include <environment/vertex_se3.h>

namespace XBot { namespace Edge {
    
    class Edge_SE3 : public g2o::BaseBinaryEdge<7, XBot::Vertex::SE3, XBot::Vertex::Vertex_SE3, XBot::Vertex::Vertex_SE3>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        Edge_SE3();
        
        void computeError()
        {
            
        }
        
    private:
        
    }; } }