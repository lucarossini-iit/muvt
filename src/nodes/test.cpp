#include <iostream>
#include <cmath>
#include <memory>
#include <boost/graph/graph_concepts.hpp>

#include <ros/ros.h>

#include <g2o/config.h>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include <g2o/core/optimization_algorithm_levenberg.h>
#include "g2o/core/optimization_algorithm_factory.h"


#include <environment/edge_xyz.h>
#include <simulator/simulator.h>
#include <environment/types_planning.h>

using namespace XBot::HyperGraph;
using namespace g2o;

// It seems that all the new types must be defined in g2o namespace!

int main(int argc, char** argv)
{    
    std::cout << "starting!" << std::endl;
    Eigen::VectorXd start(3), goal(3);
    start << 0.3, 0.25, 0.0;
    goal << 0.3, -0.25, 0.0;
    Simulator simulator(100, start, goal, Simulator::ScenarioType::XYZ);
    SparseOptimizer optimizer;
    
    auto linearSolver = g2o::make_unique<LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    g2o::OptimizationAlgorithm *algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
    
    optimizer.setVerbose(true);
    
//     g2o::OptimizationAlgorithmProperty solverProperty;
//     optimizer.setAlgorithm(
//       g2o::OptimizationAlgorithmFactory::instance()->construct("lm_dense", solverProperty));
    
    optimizer.setAlgorithm(algorithm);
    
//     std::cout << "adding obstacles..." << std::endl;
//     Eigen::Vector3d obs;
//     obs << 0.5, 0.1, 0.0;
//     simulator.addObstacle(obs);
//     std::cout << "added obstacles!" << std::endl;
    
    std::cout << "adding vertices to optimizer..." << std::endl;
    PointGrid points = simulator.getVertices();   
    for (int i = 0; i < points.size(); i++)
    {       
        auto v = new VertexPointXYZ;
        v->setEstimate(points[i].point);
        v->setId(i);
        if (i == 0 || i == points.size() - 1)
            v->setFixed(true);
        optimizer.addVertex(v);
    }
    std::cout << "done!" << std::endl;
    
//     std::cout << "adding obstacles to optimizer..." << std::endl;
//     ObstacleGrid obstacles = simulator.getObstacles();
//     for (int i = 0; i < obstacles.size(); i++)
//     {
//         auto v = new VertexPointXYZ;
//         v->setEstimate(obstacles[i].position);
//         v->setId(points.size()+i);
//         v->setFixed(true);
//         optimizer.addVertex(v);
//     }
//     std::cout << "done!" << std::endl;
    
//     std::cout << "adding edges to optimizer..." << std::endl;
//     for (int j = points.size(); j < points.size() + obstacles.size(); j++)
//     {
//         for (int i = 0; i < points.size(); i++)
//         {
//             auto edge = new EdgeScalarXYZ;
//             edge->vertices()[0] = optimizer.vertex(i);
//             edge->vertices()[1] = optimizer.vertex(j);
//             optimizer.addEdge(edge);
//         }
//     }   
//     std::cout << "done" << std::endl;
    
    std::cout << "adding edges to optimizer..." << std::endl;
    Eigen::Vector3d obs;
    obs << 0.5, 0.1, 0.0;
    for (int i = 0; i < points.size(); i++)
    {
        auto e = new EdgeScalarXYZ;
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->vertices()[0] = optimizer.vertex(i);
        e->setObstacle(obs);
        optimizer.addEdge(e);
    }
    std::cout << "done!" << std::endl;

    std::cout << "adding binary edges to optimizer" << std::endl;
    for (int i = 0; i < points.size() - 1; i++)
    {
        auto e = new EdgeDistance;
        e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        e->vertices()[0] = optimizer.vertex(i);
        e->vertices()[1] = optimizer.vertex(i+1);
        optimizer.addEdge(e);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(100);
    
    // Retrieve solution
    auto sol = optimizer.vertices();
    optimizer.save(std::cout);
       
    return true;
}
