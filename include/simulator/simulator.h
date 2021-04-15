#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <environment/edge_xyz.h>
namespace XBot { namespace TEB {
    
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
        Simulator(int n, double distance);
        
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