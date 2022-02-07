#include <simulator/simulator.h>

using namespace XBot::HyperGraph;
using namespace g2o;

Simulator::Simulator (int n, Eigen::VectorXd start, Eigen::VectorXd goal, Simulator::ScenarioType type):
_num_nodes(n), 
_scenario_type(type)
{
    if (_vertices.size() > 0)
        _vertices.clear();
    if (_configurations.size() > 0)
        _configurations.clear();
    
    if (_scenario_type == ScenarioType::XYZ)
    {
        Eigen::VectorXd direction = goal - start;
        for (int i = 0; i < _num_nodes; i++)
        {
            g2o::Vector3 pos;
            pos << start(0) + i*direction(0)/_num_nodes, start(1) + i*direction(1)/_num_nodes, start(2) + i*direction(2)/_num_nodes;
            
            Point p;
            p.point = pos;
            
            _vertices.push_back(p);
        }
    }
    
    if (_scenario_type == ScenarioType::ROBOTPOS)
    {
        if (goal == start)
        {
            Configuration c;
            c.q = start;
            c.n_dof = start.size();
            _configurations.push_back(c);
            
            std::cout << "CONFIGURATIONS.SIZE(): " << _configurations.size() << std::endl;
            std::cout << "CONTAINING: " << _configurations[0].q.transpose() << std::endl;
        }
        else
        {
            Eigen::VectorXd direction = goal - start;
            for (int i = 0; i < _num_nodes; i++)
            {
                Eigen::VectorXd pos(goal.size());
                for (int j = 0; j < pos.size(); j++)
                    pos(j) = start(j) + i*direction(j)/_num_nodes;
                
                Configuration c;
                c.q = pos;
                c.n_dof = pos.size(); 
                
                _configurations.push_back(c);
            }
        }
    }
}

bool Simulator::addObstacle (Eigen::Vector3d position)
{
    Obstacle obs;
    obs.position = position;
    
    _obstacles.push_back(obs);

     return true;
}

void Simulator::clear()
{
    _vertices.clear();
    _obstacles.clear();
    _configurations.clear();
}

PointGrid Simulator::getVertices() const
{
    return _vertices;
}

ConfigurationGrid Simulator::getConfigurations() const
{
    return _configurations;
}


ObstacleGrid Simulator::getObstacles() const
{
    return _obstacles;
}

Simulator::ScenarioType Simulator::getScenarioType() const 
{
    return _scenario_type;
}



