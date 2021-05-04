#include <simulator/simulator.h>

using namespace XBot::HyperGraph;
using namespace g2o;

Simulator::Simulator (int n, Eigen::Vector3d start, Eigen::Vector3d goal):
_num_nodes(n)
{
    if (_vertices.size() > 0)
        _vertices.clear();
    
    Eigen::Vector3d direction = goal - start;
    for (int i = 0; i < _num_nodes; i++)
    {
        g2o::Vector3 pos;
        pos << start(0) + i*direction(0)/_num_nodes, start(1) + i*direction(1)/_num_nodes, start(2) + i*direction(2)/_num_nodes;
        
        Point p;
        p.point = pos;
        
        _vertices.push_back(p);
    }
        
/*    for (int i = 0; i < _num_nodes; i++)
    {
        g2o::Vector3 pos;
        pos << i*distance/_num_nodes, 0, 0;

        Point p;
        p.point = pos;
       
        _vertices.push_back(p);
    } */  
}

bool Simulator::addObstacle (Eigen::Vector3d position)
{
    Obstacle obs;
    obs.position = position;
    
    _obstacles.push_back(obs);
}

void Simulator::clear()
{
    _vertices.clear();
    _obstacles.clear();
}

PointGrid Simulator::getVertices() const
{
    return _vertices;
}

ObstacleGrid Simulator::getObstacles() const
{
    return _obstacles;
}


