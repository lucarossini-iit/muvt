#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include <ros/ros.h>
#include <ros/service.h>

#include <g2o/core/sparse_optimizer.h>

#include <simulator/simulator.h>
#include <environment/edge_xyz.h>

namespace XBot { namespace HyperGraph {

class Optimizer {
public:
    Optimizer();

private:
    void init_load_simulator();

    ros::NodeHandle _nhpr;

    Simulator::Ptr _simulator;

    g2o::SparseOptimizer _optimizer;
};
} }

#endif // OPTIMIZER_H
