#include <muvt_core/environment/contact/vertex_contact.h>

using namespace Muvt::HyperGraph;
using namespace g2o;

VertexContact::VertexContact():
BaseVertex<6, Contact>()
{}

bool VertexContact::read(std::istream& is) {
  return true;
}

bool VertexContact::write(std::ostream& os) const {
  return true;
}

void VertexContact::print() const
{
    std::cout << "VertexContact -- ID: " << _id << std::endl;
    std::cout << "distal link: " << _estimate.getDistalLink() << std::endl;
    std::cout << "pose: \n" << _estimate.state.pose.matrix() << std::endl;
    std::cout << "time: " << _estimate.state.time << std::endl;
}

void VertexContact::setToOriginImpl()
{
    _estimate.state.pose.setIdentity();
}

bool VertexContact::setEstimateDataImpl(const double* est)
{
    Utils::vectorToContact(est, _estimate);
    return true;
}

bool VertexContact::getEstimateData(double* est) const
{
    Utils::contactToVector(_estimate, est);
    return true;
}

bool VertexContact::setMinimalEstimateDataImpl(const double *est)
{
    Utils::minimalVectorToContact(est, _estimate);
    return true;
}

bool VertexContact::getMinimalEstimateDataImpl(double *est)
{
    Utils::contactToMinimalVector(_estimate, est);
    return true;
}

void VertexContact::oplusImpl(const double* update)
{
    Contact increment;
    Utils::minimalVectorToContact(update, increment);
    _estimate.state.pose.translation() = _estimate.state.pose.translation() + increment.state.pose.translation();
    _estimate.state.pose.linear() = _estimate.state.pose.linear() * increment.state.pose.linear();
}

void Utils::vectorToContact(const double* v, Contact& c)
{
    c.state.pose.translation() << v[0], v[1], v[2];
    c.state.pose.linear() = Eigen::Quaternion<double>(v[6], v[3], v[4], v[5]).toRotationMatrix();
}

void Utils::contactToVector(const Contact c, double* v)
{
    Eigen::Map<Eigen::Matrix<double, 7, 1>> est(v);
    est.head(3) = c.state.pose.translation();
    Eigen::Quaternion<double> q(c.state.pose.linear());
    est.tail(4) << q.coeffs().x(), q.coeffs().y(), q.coeffs().z(), q.coeffs().w();
}

void Utils::minimalVectorToContact(const double *v, Contact &c)
{
    c.state.pose.translation() << v[0], v[1], v[2];
    double w = 1 - (std::pow(v[3], 2) + std::pow(v[4], 2) + std::pow(v[5], 2));
    if (w<0)
        c.state.pose.linear().setIdentity();
    else
    {
        w=std::sqrt(w);
        Eigen::Quaternion<double> q(w, v[3], v[4], v[5]);
        c.state.pose.linear() = q.toRotationMatrix();
    }
}

void Utils::contactToMinimalVector(const Contact c, double *v)
{
    v[0] = c.state.pose.translation().x();
    v[1] = c.state.pose.translation().y();
    v[2] = c.state.pose.translation().z();
    Eigen::Quaternion<double> q(c.state.pose.linear());
    q.normalize();
    v[3] = q.coeffs().x();
    v[4] = q.coeffs().y();
    v[5] = q.coeffs().z();
}
