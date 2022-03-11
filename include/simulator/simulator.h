#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <environment/joint/edge_robot_pos.h>
#include <environment/joint/vertex_robot_pos.h>

namespace XBot { namespace HyperGraph {
    
    struct Point {
        g2o::Vector3 point;
    };
    
    struct Obstacle {
        g2o::Vector3 position;
    };
    
    struct Configuration {
        Eigen::VectorXd q;
        int n_dof;
    };
    
    using PointGrid = std::vector<Point>;
    using ObstacleGrid = std::vector<Obstacle>;
    using ConfigurationGrid = std::vector<Configuration>;
    
    class Simulator {
    public:
        enum class ScenarioType {XYZ, ROBOTPOS};
        
        typedef std::shared_ptr<Simulator> Ptr;
        Simulator(int n, Eigen::VectorXd start, Eigen::VectorXd goal, ScenarioType type);
        ~Simulator() = default;
        
        bool addObstacle(Eigen::Vector3d position);
        
        void clear();
        
        PointGrid getVertices() const;
        ConfigurationGrid getConfigurations() const;
        ObstacleGrid getObstacles() const;
        
        ScenarioType getScenarioType() const;
    
    private:
        unsigned int _num_nodes;
        
        PointGrid _vertices;
        ObstacleGrid _obstacles;
        ConfigurationGrid _configurations;
        
        ScenarioType _scenario_type;
    };
    
    
} }

#endif
