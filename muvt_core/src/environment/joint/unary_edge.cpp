#include <muvt_core/environment/joint/unary_edge.h>

using namespace Muvt::HyperGraph;
using namespace g2o;

UnaryEdge::UnaryEdge(XBot::ModelInterface::Ptr model):
BaseUnaryEdge<-1, Eigen::VectorXd, VertexRobotPos>(),
_model(model)
{}
