#include <environment/edge_xyz.h>

using namespace g2o;

// EdgeScalarXYZ::EdgeScalarXYZ():
// BaseBinaryEdge<1, double, VertexPointXYZ, VertexPointXYZ>()
// {}
// 
// bool EdgeScalarXYZ::read(std::istream& is)
//   {
//     double p;
//     is >> p;
//     setMeasurement(p);
// 
//     return true;
//   }
// 
//   bool EdgeScalarXYZ::write(std::ostream& os) const
//   {
//     double p = measurement();
//     os << p;
//     
//     return os.good();
//   }
// 
// 
// void EdgeScalarXYZ::computeError() 
// {
//     const VertexPointXYZ* v1 = static_cast<const VertexPointXYZ*>(_vertices[0]);
//     const VertexPointXYZ* v2 = static_cast<const VertexPointXYZ*>(_vertices[1]);
//     
//     double distance = sqrt(pow(v1->estimate()(0) - v2->estimate()(0), 2) + pow(v1->estimate()(1) - v2->estimate()(1), 2) + pow(v1->estimate()(2) - v2->estimate()(2), 2));
//     double eps = 0.1;
//     double S = 0.05;
//     double r = 0.2;
//     int n = 2;
//     double value = pow((-distance-(-r-eps))/S, n);
//     if (-distance > -r - eps) 
//         _error << value;
//     else
//         _error << 0;
//     if (_error(0) > 0 )
//         std::cout <<"from: " << v1->estimate().transpose() << "   to: " << v2->estimate().transpose() << "            " << _error << std::endl;
// }
// 
// void EdgeScalarXYZ::linearizeOplus() 
// {
// //     g2o::BaseFixedSizedEdge< int ( 1 ), double, g2o::VertexPointXYZ, g2o::VertexPointXYZ >::linearizeOplus();
//     _jacobianOplusXi(0) = 1;
//     _jacobianOplusXj(0) = 0;
// }

// BASE UNARY TEST

EdgeScalarXYZ::EdgeScalarXYZ():
BaseUnaryEdge<1, double, VertexPointXYZ>()
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
    
    double distance = sqrt(pow(v1->estimate()(0) - _obstacle(0), 2) + pow(v1->estimate()(1) - _obstacle(1), 2) + pow(v1->estimate()(2) - _obstacle(2), 2));
    double eps = 0.1;
    double S = 0.05;
    double r = 0.2;
    int n = 2;
    double value = pow((-distance-(-r-eps))/S, n);
    if (-distance > -r - eps) 
        _error << value;
    else
        _error << 0;
    
//    if (_error(0) > 0)
//        std::cout << "vertex: " << v1->estimate().transpose() << "   \nw.r.t. obs: " << _obstacle.transpose() << "   ->  " << _error << std::endl;
}

EdgeDistance::EdgeDistance():
BaseBinaryEdge<1, double, VertexPointXYZ, VertexPointXYZ>()
{}

bool EdgeDistance::read(std::istream& is)
{
    double p;
    is >> p;
    setMeasurement(p);

    return true;
}

bool EdgeDistance::write(std::ostream& os) const
{
    double p = measurement();
    os << p;

    return os.good();
}

void EdgeDistance::computeError()
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
        _error << value;
    else
        _error << 0;

    if (_error(0) > 0)
        std::cout << "vertex: " << v1->estimate().transpose() << "   \nw.r.t. obs: " << v2->estimate().transpose() << "   ->  " << _error << std::endl;
}








