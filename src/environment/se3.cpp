#include <environment/se3.h>

using namespace XBot::Vertex;

SE3::SE3(const SE3& obj)
{
    _t = obj._t;
    _q = obj._q;
}

SE3 SE3::operator= ( const SE3& obj ) 
{
    _t = obj._t;
    _q = obj._q;
    
    return *this;
}

SE3 SE3::operator* ( const SE3& obj ) const
{
    SE3 result(*this);
    result._t += _q.toRotationMatrix() * obj._t;
    result._q *= obj._q;
    result._q.normalize();
    
    return result;
}


Eigen::Quaternion<double> SE3::rotation() const 
{
    return _q;
}

Eigen::Vector3d SE3::translation() const
{
    return _t;
}

void SE3::fromVector ( const Eigen::VectorXd& v ) 
{
    if (v.size() != 7)
    {
        std::cerr << "Wrong input vector size! Setting to origin..." << std::endl;
        *this = SE3();
    }
    else
    {
        _t << v[0], v[1], v[2];
        _q = Eigen::Quaternion<double>(v[3], v[4], v[5], v[6]);
    }
}

Eigen::VectorXd SE3::toVector() const 
{
    Eigen::VectorXd v(7);
    v(0) = _t(0);
    v(1) = _t(1);
    v(2) = _t(2);
    v(3) = _q.x();
    v(4) = _q.y();
    v(5) = _q.z();
    v(6) = _q.w();
    
    return v;
}









