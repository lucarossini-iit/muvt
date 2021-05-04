#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <environment/edge_xyz.h>
namespace XBot { namespace HyperGraph {
    
    struct Point {
        g2o::Vector3 point;
    };
    
    struct Obstacle {
        g2o::Vector3 position;
    };
    
    using PointGrid = std::vector<Point>;
    using ObstacleGrid = std::vector<Obstacle>;
    
    class Simulator {
    public:
        typedef std::shared_ptr<Simulator> Ptr;
        Simulator(int n, Eigen::Vector3d start, Eigen::Vector3d goal);
        
        bool addObstacle(Eigen::Vector3d position);
        
        void clear();
        
        PointGrid getVertices() const;
        ObstacleGrid getObstacles() const;
    
    private:
        unsigned int _num_nodes;
        
        PointGrid _vertices;
        ObstacleGrid _obstacles;
    };
    
    
} }
