#include <muvt_core/optimizer/optimizer_contact.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

using namespace g2o;
using namespace Muvt::HyperGraph;

OptimizerContact::OptimizerContact()
{
    init_optimizer();
}

void OptimizerContact::init_optimizer()
{
    auto linearSolver = g2o::make_unique<LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>>();
    auto blockSolver = g2o::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    g2o::OptimizationAlgorithm *algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    _optimizer.setVerbose(false);
    _optimizer.setAlgorithm(algorithm);
}

void OptimizerContact::setVertices(const std::vector<OptimizableGraph::Vertex*> vertices)
{
    if (!_vertices.empty())
        _vertices.clear();

    for (auto vertex : vertices)
        _vertices.push_back(vertex);
}

void OptimizerContact::setEdges(const std::vector<OptimizableGraph::Edge*> edges)
{
    if (!_edges.empty())
        _edges.clear();

    for (auto edge : edges)
        _edges.push_back(edge);
}

void OptimizerContact::clear()
{
    _optimizer.clear();
    _optimizer.clearParameters();
}

void OptimizerContact::update()
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

void OptimizerContact::solve()
{
    _optimizer.initializeOptimization();
    _optimizer.optimize(10);
}

void OptimizerContact::getFootsteps(std::vector<Contact>& footsteps)
{
    footsteps.clear();
    auto vertices = _optimizer.vertices();
    for (int i = 0; i < vertices.size(); i++)
    {
        VertexContact* v = dynamic_cast<VertexContact*>(vertices[i]);
        footsteps.push_back(v->estimate());
    }
}
