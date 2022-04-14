#include <teb_test/optimizer/optimizer_contact.h>

using namespace g2o;
using namespace XBot::HyperGraph;

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
    for (auto edge : edges)
        _edges.push_back(edge);
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
