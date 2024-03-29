#ifndef MUVT_CORE_OPTIMIZER_JOINT_H
#define MUVT_CORE_OPTIMIZER_JOINT_H

#include <g2o/config.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimizable_graph.h>

#include <muvt_core/environment/contact/vertex_contact.h>

namespace Muvt { namespace HyperGraph {

class OptimizerJoint {
public:
    OptimizerJoint();

    void setVertices(const std::vector<g2o::OptimizableGraph::Vertex*> vertices);
    std::vector<g2o::OptimizableGraph::Vertex*> getVertices() const { return _vertices; }

    void setEdges(const std::vector<g2o::OptimizableGraph::Edge*> edges);
    std::vector<g2o::OptimizableGraph::Edge*> getEdges() const { return _edges; }

    void update();

    /**
     * @brief remove all the vertices and edges
     */
    void clear();

    void solve();

private:
    void init_optimizer();

    std::vector<g2o::OptimizableGraph::Vertex*> _vertices;
    std::vector<g2o::OptimizableGraph::Edge*> _edges;

    g2o::SparseOptimizer _optimizer;

};

} }

#endif // MUVT_CORE_OPTIMIZER_JOINT_H

