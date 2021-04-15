#include <environment/edge_xyz.h>

using namespace g2o;

EdgeScalarXYZ::EdgeScalarXYZ():
BaseBinaryEdge<1, double, VertexPointXYZ, VertexPointXYZ>()
{}

bool EdgeScalarXYZ::read(std::istream& is)
  {
    double p;
    is >> p;
    setMeasurement(p);

    return true;
  }

  bool EdgeScalarXYZ::write(std::ostream& os) const
  {
    double p = measurement();
    os << p;
    
    return os.good();
  }


void EdgeScalarXYZ::computeError() 
{
    const VertexPointXYZ* v1 = static_cast<const VertexPointXYZ*>(_vertices[0]);
    const VertexPointXYZ* v2 = static_cast<const VertexPointXYZ*>(_vertices[1]);
    
    double distance = sqrt(pow(v1->estimate()(0) - v2->estimate()(0), 2) + pow(v1->estimate()(1) - v2->estimate()(1), 2) + pow(v1->estimate()(2) - v2->estimate()(2), 2));
    double eps = 0.1;
    double S = 0.05;
    double r = 0.2;
    int n = 2;
    double value = pow((-distance-(-r-eps))/S, n);
    if (-distance > -r - eps) 
        _error(0) = value;
    else
        _error(0) = 0;
    
    std::cout << "vertex[0]: " << v1->estimate().transpose() << "   vertex[1]: " << v2->estimate().transpose() << "   error: " << _error << std::endl;
}



