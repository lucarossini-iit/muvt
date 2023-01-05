#include <muvt_core/optimizer/optimizer_joint.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

using namespace g2o;
using namespace Muvt::HyperGraph;

OptimizerJoint::OptimizerJoint()
{
    init_optimizer();
}

void OptimizerJoint::init_optimizer()
{
    auto linearSolver = g2o::make_unique<LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    auto blockSolver = g2o::make_unique<BlockSolverX>(std::move(linearSolver));
    g2o::OptimizationAlgorithm *algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    _optimizer.setVerbose(false);
    _optimizer.setAlgorithm(algorithm);
}

void OptimizerJoint::setVertices(const std::vector<OptimizableGraph::Vertex*> vertices)
{
    if (!_vertices.empty())
        _vertices.clear();

    for (auto vertex : vertices)
        _vertices.push_back(vertex);
}

void OptimizerJoint::setEdges(const std::vector<OptimizableGraph::Edge*> edges)
{
    if (!_edges.empty())
        _edges.clear();

    for (auto edge : edges)
        _edges.push_back(edge);
}

void OptimizerJoint::clear()
{
    _optimizer.clear();
    _optimizer.clearParameters();
}

void OptimizerJoint::update()
{
    for (auto v : _vertices)
    {
        _optimizer.addVertex(v);
    }

    for (auto e : _edges)
    {
        _optimizer.addEdge(e);
    }

    std::cout << "\033[1;32m[planner_executor] \033[0m" << "\033[32m" << "loaded " << _vertices.size() << " vertices, and " << _edges.size() << "edges!" <<  "\033[0m" << std::endl;
}

void OptimizerJoint::solve()
{
    _optimizer.initializeOptimization();
    _optimizer.optimize(10);
}

