#ifndef EDGE_SCALAR_XYZ
#define EDGE_SCALAR_XYZ

#include <g2o/config.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/types/slam3d/edge_pointxyz.h>

namespace g2o {
    
//     class EdgeScalarXYZ : public BaseBinaryEdge<1, double, VertexPointXYZ, VertexPointXYZ>{
//     public:
//         EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//         EdgeScalarXYZ();
//         
//         virtual bool read(std::istream& is);
//         virtual bool write(std::ostream& os) const;
//         
//         virtual void setMeasurement(const double& m)
//         {
//             _measurement = m;
//         }
//         
//         void computeError();
//         
//         void linearizeOplus();
//                                                 
//     }; 
    
    
    // UNARY EDGE TEST
    class EdgeScalarXYZ : public BaseUnaryEdge<1, double, VertexPointXYZ>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeScalarXYZ();
        
        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;
        
        void setObstacle(Eigen::Vector3d ob)
        {
            _obstacle = ob;
        }
        
        void computeError();
        
    private:
        Eigen::Vector3d _obstacle;
    };

    class EdgeDistance : public BaseBinaryEdge<1, double, VertexPointXYZ, VertexPointXYZ>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        EdgeDistance();

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        void computeError();
    };
    
} 
    
#endif
